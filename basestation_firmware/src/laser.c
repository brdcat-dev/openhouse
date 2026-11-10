/* Lasers: ISL58303 dual laser-diode driver + per-revolution sweep window.
 * Stock: isl58303_init @0x9438, reg read @0x95B8/0x9588, reg write @0x96EC,
 * verified write @0x9810, channel regs @0x933C, laser_init @0x9A00,
 * laser_update_task @0x9DE0, laser_set_enable @0x9D28, event cb @0x9B5C.
 *
 *   laser 0 = ISL channel 1, enable pin P1_12, swept by rotor 0
 *   laser 1 = ISL channel 0, enable pin P0_16, swept by rotor 1
 *   channel regs: ch*0x10 + 0x12 bias, +0x13 (0x50), +0x16 (0x40), +0x17 drive current
 * Bus: P1_22 SDIO, P1_20 SCLK (idle high, latch on rising edge), P1_1 CS (active
 * high). Shared with the display shift register, which only latches on its own
 * latch pin, so ISL traffic never disturbs the display.
 *
 * SAFETY: a stationary beam is not eye safe. Lasers are only switched on by the
 * per-revolution 'on' event of a running, locked rotor, are always switched off
 * ~124 degrees later, and are additionally forced off by laser_safety_check()
 * (rotor control, 60 Hz) and laser_watchdog() (SysTick, 50 Hz). */
#include "laser.h"
#include "rotor.h"
#include "timebase.h"
#include "timers.h"
#include "params.h"
#include "board.h"
#include "sys.h"

#define ISL_SDIO    PIN(1, 22)
#define ISL_SCLK    PIN(1, 20)
#define ISL_CS      PIN(1, 1)

static const int laser_pin[2] = { PIN_LASER0_EN, PIN_LASER1_EN };

/* power sense (stock laser_apc_result @0x988C): laser 0 -> AD2 (P0_13),
 * laser 1 -> AD0 (P0_11); photodiode amplifier gain via a serial latch on
 * P1_2 data / P1_20 clock / P1_3 strobe (stock @0xC1BC/0x2F44, descriptor
 * @0xDC9C: one 8-bit field, MSB first) */
#define ADC_CR      REG(0x4001C000)
#define ADC_DR(ch)  REG(0x4001C010 + 4 * (ch))
static const uint8_t adc_ch[2] = { 2, 0 };
#define FPD_DATA    PIN(1, 2)
#define FPD_CLK     PIN(1, 20)
#define FPD_STROBE  PIN(1, 3)

volatile bool apc_enabled = true;       /* stock: per laser.apc parameter */
volatile uint16_t apc_adc[2];
volatile float apc_detected[2], apc_power[2];
static uint8_t boot_current[2];
static volatile bool current_dirty;
volatile bool laser_fault;
static const int isl_channel[2] = { 1, 0 };

volatile bool lasers_requested = true;  /* console override; stock: always */
static bool isl_ok;
static uint8_t isl_id;
volatile uint32_t laser_on_count[2], laser_kill_count;

/* ---- ISL58303 3-wire bus ---- */
static inline void clk_pulse(void) { gpio_toggle(ISL_SCLK); gpio_toggle(ISL_SCLK); }

static void isl_begin(int rw)
{
    gpio_clr(ISL_CS);
    gpio_output(ISL_SDIO, true);
    gpio_set(ISL_SCLK);
    gpio_set(ISL_CS);
    gpio_write(ISL_SDIO, rw);       /* R/W bit */
    clk_pulse();
}

static void isl_write(uint8_t reg, uint8_t val)     /* stock @0x96EC */
{
    uint32_t s = irq_save();        /* display refresh shares the bus */
    isl_begin(0);
    for (int i = 6; i >= 0; i--) { gpio_write(ISL_SDIO, reg & (1u << i)); clk_pulse(); }
    for (int i = 7; i >= 0; i--) { gpio_write(ISL_SDIO, val & (1u << i)); clk_pulse(); }
    gpio_clr(ISL_CS);
    irq_restore(s);
}

static uint8_t isl_read1(uint8_t reg)               /* stock @0x95B8 */
{
    uint32_t s = irq_save();
    uint8_t v = 0;
    isl_begin(1);
    for (int i = 6; i >= 0; i--) { gpio_write(ISL_SDIO, reg & (1u << i)); clk_pulse(); }
    gpio_input(ISL_SDIO);
    clk_pulse();                    /* turnaround */
    for (int i = 7; i >= 0; i--) {
        if (gpio_read(ISL_SDIO)) v |= 1u << i;
        clk_pulse();
    }
    gpio_clr(ISL_CS);
    gpio_output(ISL_SDIO, false);
    irq_restore(s);
    return v;
}

uint8_t isl_read(uint8_t reg) { isl_read1(reg); return isl_read1(reg); }   /* stock @0x9588 */

static bool isl_set(uint8_t reg, uint8_t val)       /* stock @0x9810: verify, 5 tries */
{
    for (int i = 0; i < 5; i++) {
        isl_write(reg, val);
        if (isl_read(reg) == val) return true;
    }
    return false;
}

static bool isl_channel_reg(int ch, int which, uint8_t val)     /* stock @0x933C */
{
    static const uint8_t off[4] = { 0x12, 0x16, 0x17, 0x13 };
    return isl_set(ch * 0x10 + off[which], val);
}

/* ---- enable pins ---- */
static inline void laser_pin_write(int n, bool on) { gpio_write(laser_pin[n], on); }
void lasers_off(void) { laser_pin_write(0, false); laser_pin_write(1, false); }

static const uint8_t rev3[8] = { 0, 4, 2, 6, 1, 5, 3, 7 };    /* stock table @0xDD04 */

static void fpd_gain_set(void)
{
    uint8_t v = rev3[P.laser_1_pwr_gain & 7] | (rev3[P.laser_0_pwr_gain & 7] << 3);
    uint32_t s = irq_save();
    for (int i = 7; i >= 0; i--) {
        gpio_write(FPD_DATA, v & (1u << i));
        gpio_toggle(FPD_CLK);
        gpio_toggle(FPD_CLK);
    }
    gpio_toggle(FPD_STROBE);
    gpio_toggle(FPD_STROBE);
    irq_restore(s);
}

static void adc_init(void)              /* stock @0x2DDC/0x2E30 */
{
    PDRUNCFG &= ~(1u << 4);
    SYSAHBCLKCTRL |= (1u << 13);
    IOCON_P0(13) = IOCON_FUNC(2);       /* AD2, analog (ADMODE=0), no pull */
    IOCON_P0(11) = IOCON_FUNC(2);       /* AD0 */
    ADC_CR = ((CPU_HZ / 4500000) << 8); /* ~4.4 MHz ADC clock, 10-bit */
}

static uint16_t adc_sample(int ch)
{
    ADC_CR = (ADC_CR & ~0x070000FFu) | (1u << ch) | (1u << 24);   /* start now */
    for (int i = 0; i < 400; i++) {
        uint32_t d = ADC_DR(ch);
        if (d & 0x80000000u) return (d >> 6) & 0x3FF;
    }
    return 0xFFFF;
}

bool laser_init(void)
{
    lasers_off();
    gpio_output(FPD_DATA, false);
    gpio_output(FPD_STROBE, false);
    adc_init();
    gpio_output(ISL_SDIO, false);
    gpio_output(ISL_SCLK, false);
    gpio_output(ISL_CS, false);

    isl_id = isl_read(1);
    if ((isl_id & 0xFC) != 0xE0) return isl_ok = false;     /* chip ID */
    bool ok = isl_set(7, 0x81);
    if (!(isl_read(3) & 1)) ok = false;                     /* VSO in range */
    ok &= isl_channel_reg(1, 3, 0x50) && isl_channel_reg(0, 3, 0x50);
    ok &= isl_channel_reg(1, 1, 0x40) && isl_channel_reg(0, 1, 0x40);
    /* stock laser_update_task: bias and drive current from the parameters */
    ok &= isl_channel_reg(isl_channel[0], 0, P.laser_0_bias);
    ok &= isl_channel_reg(isl_channel[1], 0, P.laser_1_bias);
    ok &= isl_channel_reg(isl_channel[0], 2, P.laser_0_current);
    ok &= isl_channel_reg(isl_channel[1], 2, P.laser_1_current);
    boot_current[0] = P.laser_0_current;
    boot_current[1] = P.laser_1_current;
    fpd_gain_set();
    return isl_ok = ok;
}

/* A channel change saves every parameter, and the drive currents in it must be the ones
 * the lasers booted with, not where power regulation has moved them: saved on each
 * change, the regulated values ratcheted 89AE90F2's currents down to ~1 over a few boots
 * (2026-11-08: laser fault, 3 mW of 30, since the window is boot +-60). An explicit
 * console `save` still saves what is set. */
void laser_boot_currents_for_save(bool saving)
{
    static uint8_t regulated[2];
    if (saving) {
        regulated[0] = P.laser_0_current;
        regulated[1] = P.laser_1_current;
        P.laser_0_current = boot_current[0];
        P.laser_1_current = boot_current[1];
    } else {
        P.laser_0_current = regulated[0];
        P.laser_1_current = regulated[1];
    }
}

bool laser_driver_ok(void) { return isl_ok; }
uint8_t laser_driver_id(void) { return isl_id; }

/* ---- conditions (stock event type 0, plus lhbs arming) ---- */
static bool laser_allowed(int n)
{
    rotor_state_t *r = &rotor[n];
    bool en = n ? P.laser_1_enable : P.laser_0_enable;
    return isl_ok && !laser_fault && lasers_requested && !P.sys_standby && emission_active() && P.laser_enable && en &&
           r->running && r->locked && rotor[0].locked && rotor[1].locked && !timebase_skip();
}

/* ---- per-revolution events on CT32B0 MR1 (rotor 0) / MR2 (rotor 1) ---- */
static enum { EV_IDLE, EV_ON, EV_MID, EV_OFF, EV_PRESYNC } ev_state[2];
static uint32_t ev_mid_time[2], ev_off_time[2], ev_presync_time[2];

/* stock laser_apc_result @0x988C, plus lhbs current limits */
static void apc_sample(int n)
{
    uint16_t x = adc_sample(adc_ch[n]);
    if (x == 0xFFFF) return;
    apc_adc[n] = x;
    float fx = (float)x, prev = apc_detected[n], b1 = P.laser_b1;
    float diff = fx - prev;
    float det = (diff <= 5.0f && diff >= -5.0f) ? (1.0f - b1) * fx + b1 * prev : fx;
    apc_detected[n] = det;
    float m = n ? P.laser_1_pwr_m : P.laser_0_pwr_m;
    float b = n ? P.laser_1_pwr_b : P.laser_0_pwr_b;
    float b2 = n ? P.laser_1_pwr_b2 : P.laser_0_pwr_b2;
    float pw = (det - b) * m + b2;
    if (pw < 0) pw = 0;
    apc_power[n] = pw;
    if (n) { P.laser_1_pwr_detected = det; P.laser_1_pwr_average = pw; }
    else   { P.laser_0_pwr_detected = det; P.laser_0_pwr_average = pw; }
    if (!P.laser_apc || !apc_enabled) return;

    float target = n ? P.laser_1_pwr : P.laser_0_pwr;
    uint8_t *cur = n ? &P.laser_1_current : &P.laser_0_current;
    /* A diode warming up needs more current for the same power: +20 faulted a healthy
     * base after a few minutes (D7B1AD76: 142 -> 162 holding 29 mW). +60, and never
     * past 230 (stock default 190; healthy units run 117-172): a power sense that
     * stopped following the current still ends here, in a fault. */
    int hi = boot_current[n] + 60, lo = boot_current[n] - 60;
    if (hi > 230) hi = 230;
    /* never below 1: the current is a u8, and from a boot current under 60 the
     * decrement below wrapped 0 to 255, past the cap, with no fault (#21) */
    if (lo < 1) lo = 1;
    if (target - pw > 1.0f) {
        if (*cur >= hi) {                   /* would exceed the safety window */
            laser_fault = true;
            lasers_off();
            return;
        }
        (*cur)++;
        current_dirty = true;
    } else if (pw - target > 1.0f && *cur > lo) {
        (*cur)--;
        current_dirty = true;
    }
}

/* main loop: push APC current changes to the ISL58303 (stock does this in
 * laser_update_task, a periodic task) */
void laser_poll(void)
{
    static uint32_t last;
    if (!current_dirty || sys_ms() - last < 100) return;
    last = sys_ms();
    current_dirty = false;
    isl_channel_reg(isl_channel[0], 2, P.laser_0_current);
    isl_channel_reg(isl_channel[1], 2, P.laser_1_current);
}

/* stock event type 3 (@0x9B5C): just before our next flash, hide it from the
 * optical receiver (only matters when slaved and emitting) */
static void arm_presync(int n)
{
    lpc_timer_t *tm = TMR_CT32B0;
    if (!emission_active()) { ev_state[n] = EV_IDLE; tm->MCR &= ~MCR_I(n + 1); return; }
    ev_state[n] = EV_PRESYNC;
    tm->MR[n + 1] = ev_presync_time[n];
    tm->IR = IR_MR(n + 1);
    tm->MCR |= MCR_I(n + 1);
}

void laser_index_edge(int n, uint32_t t, uint32_t period)
{
    lpc_timer_t *tm = TMR_CT32B0;
    uint32_t expect = timebase_period();
    ev_presync_time[n] = t + period - P.timebase_optical_presync;
    if (period < expect - expect / 100 || period > expect + expect / 100 || !laser_allowed(n)) {
        laser_pin_write(n, false);
        arm_presync(n);
        return;
    }
    float start = n ? P.laser_1_start : P.laser_0_start;
    float len = n ? P.laser_1_length : P.laser_0_length;
    uint32_t t_on = t + (uint32_t)((float)period * start / 360.0f);
    ev_mid_time[n] = t + (uint32_t)((float)period * (start + len / 2) / 360.0f);
    ev_off_time[n] = t + (uint32_t)((float)period * (start + len) / 360.0f);
    ev_state[n] = EV_ON;
    tm->MR[n + 1] = t_on;
    tm->IR = IR_MR(n + 1);
    tm->MCR |= MCR_I(n + 1);
}

void laser_match_event(int n)       /* from CT32B0_IRQHandler on MR1/MR2 */
{
    lpc_timer_t *tm = TMR_CT32B0;
    if (ev_state[n] == EV_ON) {
        if (laser_allowed(n)) {
            timebase_mask_own(true);        /* stock laser_set_enable -> cc64(1) */
            laser_pin_write(n, true);
            laser_on_count[n]++;
        }
        ev_state[n] = EV_MID;
        tm->MR[n + 1] = ev_mid_time[n];
    } else if (ev_state[n] == EV_MID) {     /* stock event type 2: sample power */
        if (gpio_read(laser_pin[n])) apc_sample(n);
        ev_state[n] = EV_OFF;
        tm->MR[n + 1] = ev_off_time[n];
    } else if (ev_state[n] == EV_OFF) {
        laser_pin_write(n, false);
        timebase_mask_own(false);
        arm_presync(n);
    } else {                                /* EV_PRESYNC */
        timebase_mask_own(true);            /* cleared again at our sync end (MR0) */
        ev_state[n] = EV_IDLE;
        tm->MCR &= ~MCR_I(n + 1);
    }
}

/* 60 Hz from rotor control: kill immediately on any loss of lock/run. */
void laser_safety_check(void)
{
    for (int n = 0; n < 2; n++)
        if (!(rotor[n].running && rotor[n].locked && rotor[0].locked && rotor[1].locked) ||
            !lasers_requested) {
            if (gpio_read(laser_pin[n])) laser_kill_count++;
            laser_pin_write(n, false);
            ev_state[n] = EV_IDLE;
        }
}

/* 50 Hz from SysTick: no index for > 2 periods = rotor stopping -> off. */
void laser_watchdog(void)
{
    uint32_t now = timebase_now(), lim = 2 * timebase_period();
    for (int n = 0; n < 2; n++)
        if (gpio_read(laser_pin[n]) && now - rotor[n].last_index > lim) {
            laser_pin_write(n, false);
            ev_state[n] = EV_IDLE;
            laser_kill_count++;
        }
}
