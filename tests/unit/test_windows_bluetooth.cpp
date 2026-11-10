#include "../../src/bluetooth/windows.cpp"
#include <gtest/gtest.h>

namespace {
using namespace lh::bluetooth;
using namespace std::chrono_literals;

winrt::Windows::Foundation::IAsyncOperation<int> Value(bool fail) {
  if (fail) throw winrt::hresult_access_denied();
  co_return 42;
}

TEST(WindowsBluetooth, PreservesOperationErrorAndDistinguishesTimeout) {
  winrt::init_apartment(winrt::apartment_type::multi_threaded);
  EXPECT_EQ(Await(Value(false), Clock::now() + 1s), 42);
  try {
    Await(Value(true), Clock::now() + 1s, "Discover services");
    FAIL() << "Expected access denied";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("Discover services 0x80070005"), std::string::npos);
  }
  try {
    Await(Value(false), Clock::now() - 1s, "Expired operation");
    FAIL() << "Expected timeout";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("Expired operation 0x800705b4"), std::string::npos);
  }
  winrt::uninit_apartment();
}

TEST(WindowsBluetooth, RejectsInvalidBatchBeforeUsingAdapter) {
  const std::array<uint64_t, 2> duplicate{1, 1};
  const std::array<uint64_t, 1> invalid{uint64_t(1) << 48};
  EXPECT_THROW(SetPowerBatch(duplicate, 0, 1s), std::invalid_argument);
  EXPECT_THROW(SetPowerBatch(invalid, 0, 1s), std::invalid_argument);
  EXPECT_THROW(SetPowerBatch({}, 3, 1s), std::invalid_argument);
}
}  // namespace
