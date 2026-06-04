// CMPLocalPcrNotifyTests.cpp
//
// Gap 3.5 — host-as-CMP-target local-PCR change notify. Verifies that a
// successful inbound peer lock (compare_swap) to one of our local PCRs invokes
// the registered listener with Apple `pcrModified(plugType, plugNum, newValue)`
// semantics, and that it does NOT fire on a failed compare or an out-of-range
// address. The underlying store/compare is covered by LocalPcrRegisterFileTests;
// this exercises the CMPClient notify seam wired by ASFWDriver's lock responder.

#include <gtest/gtest.h>

#include <span>

#include "ASFWDriver/Protocols/AVC/CMP/CMPClient.hpp"

// Minimal IFireWireBusOps stub. CMPClient requires a bus reference, but the
// local-PCR read/compare-swap paths under test are pure register-file operations
// that never touch the bus, so every method is an inert default. Defined inside
// ASFW::Async so the signatures mirror the interface header verbatim.
namespace ASFW::Async {
class NullBusOps : public IFireWireBusOps {
public:
    AsyncHandle ReadBlock(FW::Generation, FW::NodeId, FWAddress, uint32_t, FW::FwSpeed,
                          InterfaceCompletionCallback) override { return {}; }
    AsyncHandle WriteBlock(FW::Generation, FW::NodeId, FWAddress, std::span<const uint8_t>,
                           FW::FwSpeed, InterfaceCompletionCallback) override { return {}; }
    AsyncHandle Lock(FW::Generation, FW::NodeId, FWAddress, FW::LockOp, std::span<const uint8_t>,
                     uint32_t, FW::FwSpeed, InterfaceCompletionCallback) override { return {}; }
    bool Cancel(AsyncHandle) override { return false; }
};
}  // namespace ASFW::Async

namespace {

using ASFW::CMP::CMPClient;
using Reg = ASFW::CMP::LocalPcrRegisterFile::Reg;
using ASFW::Async::NullBusOps;

constexpr uint32_t kIPCR0 = ASFW::CMP::PCRRegisters::kIPCRBase;  // iPCR[0]

TEST(CMPLocalPcrNotify, FiresOnSuccessfulInboundSwapWithPcrModifiedArgs) {
    NullBusOps bus;
    CMPClient cmp(bus);

    const auto cur = cmp.ReadLocalPcr(kIPCR0);
    ASSERT_TRUE(cur.has_value());

    int fired = 0;
    Reg gotType{};
    uint8_t gotNum = 0xFF;
    uint32_t gotVal = 0;
    cmp.SetPcrChangeListener([&](Reg type, uint8_t num, uint32_t val) {
        ++fired;
        gotType = type;
        gotNum = num;
        gotVal = val;
    });

    const uint32_t desired = *cur ^ 0x00010000u;  // flip a bit so the value changes
    bool swapped = false;
    const auto old = cmp.CompareSwapLocalPcr(kIPCR0, *cur, desired, &swapped);

    ASSERT_TRUE(old.has_value());
    EXPECT_EQ(*old, *cur);
    EXPECT_TRUE(swapped);
    EXPECT_EQ(fired, 1);
    EXPECT_EQ(gotType, Reg::kInputPlug);
    EXPECT_EQ(gotNum, 0u);
    EXPECT_EQ(gotVal, desired);
}

TEST(CMPLocalPcrNotify, DoesNotFireOnFailedCompare) {
    NullBusOps bus;
    CMPClient cmp(bus);

    const auto cur = cmp.ReadLocalPcr(kIPCR0);
    ASSERT_TRUE(cur.has_value());

    int fired = 0;
    cmp.SetPcrChangeListener([&](Reg, uint8_t, uint32_t) { ++fired; });

    bool swapped = true;
    const uint32_t wrongExpected = *cur + 1u;  // will not match the stored value
    const auto old = cmp.CompareSwapLocalPcr(kIPCR0, wrongExpected, 0x1234u, &swapped);

    ASSERT_TRUE(old.has_value());  // prior value still returned
    EXPECT_EQ(*old, *cur);
    EXPECT_FALSE(swapped);         // not stored
    EXPECT_EQ(fired, 0);           // and no notify
}

TEST(CMPLocalPcrNotify, DoesNotFireForAddressOutsidePcrRange) {
    NullBusOps bus;
    CMPClient cmp(bus);

    int fired = 0;
    cmp.SetPcrChangeListener([&](Reg, uint8_t, uint32_t) { ++fired; });

    bool swapped = true;
    const auto old = cmp.CompareSwapLocalPcr(0x00000000u, 0u, 1u, &swapped);

    EXPECT_FALSE(old.has_value());
    EXPECT_FALSE(swapped);
    EXPECT_EQ(fired, 0);
}

}  // namespace
