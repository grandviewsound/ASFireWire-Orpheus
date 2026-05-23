#define private public
#include "Protocols/AVC/FCPTransport.hpp"
#undef private

#include <gtest/gtest.h>

#include <vector>

using namespace ASFW::Protocols::AVC;

namespace {

FCPFrame MakeFrame(std::initializer_list<uint8_t> bytes) {
    FCPFrame frame{};
    frame.length = bytes.size();

    size_t index = 0;
    for (uint8_t byte : bytes) {
        frame.data[index++] = byte;
    }

    return frame;
}

} // namespace

class FCPTransportResponseMatchingTests : public ::testing::Test {
protected:
    FCPTransport transport_;

    void SetPending(std::initializer_list<uint8_t> commandBytes) {
        transport_.pending_ = std::make_unique<FCPTransport::OutstandingCommand>();
        transport_.pending_->command = MakeFrame(commandBytes);
    }

    bool Validate(std::initializer_list<uint8_t> responseBytes) {
        std::vector<uint8_t> response(responseBytes);
        return transport_.ValidateResponse(response);
    }
};

TEST_F(FCPTransportResponseMatchingTests, QuerySyncPlugReconnectMatchesAppleAcceptedResponse) {
    // From working Monterey Apple capture:
    //   CMD: 02 ff 1a 0f ff 00 60 07
    //   RSP: 0c ff 1a 30 ff 00 60 07
    SetPending({0x02, 0xff, 0x1a, 0x0f, 0xff, 0x00, 0x60, 0x07});

    EXPECT_TRUE(Validate({0x0c, 0xff, 0x1a, 0x30, 0xff, 0x00, 0x60, 0x07}));
}

TEST_F(FCPTransportResponseMatchingTests, QuerySyncPlugReconnectRejectsWrongTargetTuple) {
    SetPending({0x02, 0xff, 0x1a, 0x0f, 0xff, 0x80, 0x60, 0x07});

    EXPECT_FALSE(Validate({0x0c, 0xff, 0x1a, 0x30, 0xff, 0x81, 0x60, 0x07}));
}

TEST_F(FCPTransportResponseMatchingTests, SignalSourceStatusMatchesAppleTopologyResponse) {
    // From working Monterey Apple capture:
    //   CMD: 01 ff 1a ff ff fe 08 00
    //   RSP: 0c ff 1a 70 60 01 08 00
    SetPending({0x01, 0xff, 0x1a, 0xff, 0xff, 0xfe, 0x08, 0x00});

    EXPECT_TRUE(Validate({0x0c, 0xff, 0x1a, 0x70, 0x60, 0x01, 0x08, 0x00}));
}

TEST_F(FCPTransportResponseMatchingTests, SignalSourceStatusRejectsWrongDestinationTuple) {
    SetPending({0x01, 0xff, 0x1a, 0xff, 0xff, 0xfe, 0x08, 0x00});

    EXPECT_FALSE(Validate({0x0c, 0xff, 0x1a, 0x70, 0x60, 0x01, 0x08, 0x01}));
}

TEST_F(FCPTransportResponseMatchingTests,
       SignalSourceStatusWithZeroFOperandAcceptsRewrittenSourceBytes) {
    // Regression for fix 76: panel-style sync STATUS read uses operand[0]=0x0F
    // (the standard "all-ones-upper-nibble" STATUS sentinel). The device
    // rewrites bytes 4..5 with the actual signal-source identifier and only
    // echoes the destination tuple in bytes 6..7. Validator must NOT treat
    // byte 3 = 0x0F as SPECIFIC INQUIRY — it should branch on CTYPE instead.
    //   CMD: 01 ff 1a 0f ff ff 60 07
    //   RSP: 0c ff 1a 10 ff 87 60 07
    SetPending({0x01, 0xff, 0x1a, 0x0f, 0xff, 0xff, 0x60, 0x07});

    EXPECT_TRUE(Validate({0x0c, 0xff, 0x1a, 0x10, 0xff, 0x87, 0x60, 0x07}));
}
