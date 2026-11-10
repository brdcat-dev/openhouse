/* Parameter store - reimplementation of stock common/utils/cparam.c.
 *
 * EEPROM partition = {u32 "CPRM", u32 version, u32 crc32(payload)} + payload,
 * payload = every non-runtime parameter in table order, raw little-endian.
 *
 *   stock cparam_scan        @0x6F0C -> part_scan()
 *   stock cparam_load        @0x6ACC -> params_load()
 *   stock cparam_load_raw    @0x6C08 -> payload_unpack()
 *   stock cparam_save        @0x740C -> params_save()
 *   stock cparam_write_part  @0x6E78 -> part_write()
 *   stock crc32              @0x7C5C -> crc32() (zlib polynomial)
 */
#include <string.h>
#include "params.h"
#include "iap.h"

params_t P;

#define P_DESC(field, name, pt, ctype, size, flags, dflt, en, desc) \
    { name, &P.field, pt, size, flags },
const pdesc_t param_desc[] = { PARAM_LIST(P_DESC) };
const unsigned param_count = sizeof param_desc / sizeof param_desc[0];

/* Defaults come straight from the stock default strings (see params_table.h). */
#define P_DEF_SCALAR(field, dflt)   P.field = dflt;
#define P_DEF(field, name, pt, ctype, size, flags, dflt, en, desc) P_DEF_##pt(field, dflt)
#define P_DEF_PT_BOOL     P_DEF_SCALAR
#define P_DEF_PT_U8       P_DEF_SCALAR
#define P_DEF_PT_U16      P_DEF_SCALAR
#define P_DEF_PT_U32      P_DEF_SCALAR
#define P_DEF_PT_I8       P_DEF_SCALAR
#define P_DEF_PT_I16      P_DEF_SCALAR
#define P_DEF_PT_I32      P_DEF_SCALAR
#define P_DEF_PT_FLOAT    P_DEF_SCALAR
#define P_DEF_PT_ENUM     P_DEF_SCALAR
#define P_DEF_PT_STRING(field, dflt)   strncpy(P.field, dflt, sizeof P.field);
#define P_DEF_PT_RESERVED(field, dflt) memset(P.field, 0, sizeof P.field);

void params_defaults(void)
{
    memset(&P, 0, sizeof P);
    PARAM_LIST(P_DEF)
}

/* ---- partitions ---- */
#define CPRM_MAGIC  0x4D525043u     /* "CPRM" */
enum { PART_USER = 0, PART_FACTORY = 1 };
enum { PS_UNKNOWN, PS_VALID, PS_EMPTY, PS_CRC_BAD, PS_INVALID };

typedef struct {
    uint16_t addr;
    uint8_t  type;
    uint8_t  status;
    uint32_t version;
} part_t;

/* Stock list lives in compressed RW data; addresses/types inferred from the
 * EEPROM contents (two alternating user saves + factory). */
static part_t parts[] = {
    { 0x000, PART_USER,    PS_UNKNOWN, 0 },
    { 0x300, PART_USER,    PS_UNKNOWN, 0 },
    { 0x600, PART_FACTORY, PS_UNKNOWN, 0 },
};
#define NPARTS (int)(sizeof parts / sizeof parts[0])

typedef struct { uint32_t magic, version, crc; } part_hdr_t;

static uint32_t crc32(uint32_t crc, const uint8_t *p, unsigned n)
{
    crc = ~crc;
    while (n--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1));
    }
    return ~crc;
}

unsigned params_payload_size(void)
{
    unsigned n = 0;
    for (unsigned i = 0; i < param_count; i++)
        if (!(param_desc[i].flags & PF_RUNTIME)) n += param_desc[i].size;
    return n;
}

static uint8_t payload[600];

static void payload_pack(void)
{
    uint8_t *o = payload;
    for (unsigned i = 0; i < param_count; i++) {
        const pdesc_t *d = &param_desc[i];
        if (d->flags & PF_RUNTIME) continue;
        memcpy(o, d->ptr, d->size);
        o += d->size;
    }
}

static void payload_unpack(void)
{
    const uint8_t *s = payload;
    for (unsigned i = 0; i < param_count; i++) {
        const pdesc_t *d = &param_desc[i];
        if (d->flags & PF_RUNTIME) continue;
        memcpy(d->ptr, s, d->size);
        s += d->size;
    }
}

static void part_scan(void)
{
    unsigned n = params_payload_size();
    for (int i = 0; i < NPARTS; i++) {
        part_hdr_t h;
        parts[i].version = 0;
        if (iap_eeprom_read(parts[i].addr, &h, sizeof h) ||
            iap_eeprom_read(parts[i].addr + sizeof h, payload, n)) {
            parts[i].status = PS_INVALID;
            continue;
        }
        if (h.magic == CPRM_MAGIC) {
            if (crc32(0, payload, n) == h.crc) {
                parts[i].status = PS_VALID;
                parts[i].version = h.version;
            } else {
                parts[i].status = PS_CRC_BAD;
            }
        } else {
            /* stock: "empty" if the whole region is all 0x00 or all 0xFF */
            bool all00 = true, allff = true;
            const uint8_t *b = (const uint8_t *)&h;
            for (unsigned k = 0; k < sizeof h; k++) { all00 &= b[k] == 0; allff &= b[k] == 0xFF; }
            for (unsigned k = 0; k < n; k++) { all00 &= payload[k] == 0; allff &= payload[k] == 0xFF; }
            parts[i].status = (all00 || allff) ? PS_EMPTY : PS_INVALID;
        }
    }
}

static cp_result_t load_type(int type, int *out)
{
    int best = -1;
    uint32_t best_ver = 0;
    for (int i = 0; i < NPARTS; i++)
        if (parts[i].status == PS_VALID && parts[i].type == type && parts[i].version > best_ver) {
            best_ver = parts[i].version & 0xFFFF;   /* stock keeps only 16 bits */
            best = i;
        }
    if (best < 0) return CP_BAD_PARTITION;
    if (iap_eeprom_read(parts[best].addr + sizeof(part_hdr_t), payload, params_payload_size()))
        return CP_EEPROM_READ;
    payload_unpack();
    if (out) *out = best;
    return CP_OK;
}

static cp_result_t part_write(int i, uint32_t version)
{
    unsigned n = params_payload_size();
    payload_pack();
    if (iap_eeprom_write(parts[i].addr + sizeof(part_hdr_t), payload, n)) return CP_EEPROM_WRITE;
    /* stock re-reads the payload from EEPROM to compute the CRC */
    if (iap_eeprom_read(parts[i].addr + sizeof(part_hdr_t), payload, n)) return CP_EEPROM_READ;
    part_hdr_t h = { CPRM_MAGIC, version, crc32(0, payload, n) };
    if (iap_eeprom_write(parts[i].addr, &h, sizeof h)) return CP_EEPROM_WRITE;
    parts[i].version = version;
    parts[i].status = PS_VALID;
    return CP_OK;
}

static cp_result_t save_type(int type, int *out)
{
    int oldest = -1;
    uint32_t min_ver = 0xFFFFFFFFu, max_ver = 0;
    part_scan();
    for (int i = 0; i < NPARTS; i++) {
        if (parts[i].type != type) continue;
        if (parts[i].version < min_ver) { min_ver = parts[i].version; oldest = i; }
        if (parts[i].version > max_ver) max_ver = parts[i].version;
    }
    if (oldest < 0) return CP_BAD_PARTITION;
    uint32_t ver = max_ver + 1;
    if (ver == 0) ver = 1;
    if (out) *out = oldest;
    return part_write(oldest, ver);
}

cp_result_t params_save(int *out) { return save_type(PART_USER, out); }

/* Laser drive currents a healthy unit can't have (they run 117-172, stock default 190).
 * Firmware before 2026-11-08 saved the power regulation's current on every channel change,
 * which ratcheted some units down to 1: they then fault as soon as the rotors spin, since
 * regulation may only move the current 60 from where it booted. Such a value comes from
 * the factory partition instead (or the stock default), and is saved, so it's fixed for
 * good. */
static bool laser_current_sane(uint8_t c) { return c >= 80 && c <= 230; }

static void repair_laser_currents(int *out)
{
    if (laser_current_sane(P.laser_0_current) && laser_current_sane(P.laser_1_current)) return;
    uint8_t c0 = 190, c1 = 190;
    if (load_type(PART_FACTORY, 0) == CP_OK) {
        if (laser_current_sane(P.laser_0_current)) c0 = P.laser_0_current;
        if (laser_current_sane(P.laser_1_current)) c1 = P.laser_1_current;
    }
    params_defaults();
    if (load_type(PART_USER, out) != CP_OK) return;
    if (!laser_current_sane(P.laser_0_current)) P.laser_0_current = c0;
    if (!laser_current_sane(P.laser_1_current)) P.laser_1_current = c1;
    save_type(PART_USER, 0);
}

cp_result_t params_load(int *out)
{
    params_defaults();
    part_scan();
    bool any = false;
    for (int i = 0; i < NPARTS; i++)
        if (parts[i].status != PS_EMPTY) any = true;
    if (!any) {
        params_defaults();
        cp_result_t r = params_save(out);
        return r ? r : CP_LOADED_DEFAULTS;
    }
    if (load_type(PART_USER, out) == CP_OK) {
        repair_laser_currents(out);
        return CP_OK;
    }
    if (load_type(PART_FACTORY, out) == CP_OK) return CP_LOADED_FACTORY;
    params_defaults();
    return CP_NO_PARTITIONS;
}
