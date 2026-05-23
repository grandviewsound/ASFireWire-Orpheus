//
//  AVCHandler.hpp
//  ASFWDriver
//
//  Handler for AV/C Protocol API
//

#pragma once

#include <DriverKit/IOLib.h>
#include <DriverKit/OSArray.h>
#include <DriverKit/OSDictionary.h>
#include <DriverKit/OSString.h>
#include <DriverKit/OSNumber.h>
#include <memory>
// Include subunit types for static helper serialization.
#include "../../Protocols/AVC/Music/MusicSubunit.hpp" // Adjusted path: Handler is under UserClient/Handlers. Music is Protocols/AVC/Music/
#include "../../Protocols/AVC/Audio/AudioSubunit.hpp"

struct IOUserClientMethodArguments;

namespace ASFW::Protocols::AVC {
class IAVCDiscovery;
}

namespace ASFW::UserClient {

/**
 * @brief Handler for AV/C protocol queries
 *
 * Provides GUI access to discovered AV/C units and their subunits.
 * Serializes AV/C unit information from AVCDiscovery into wire format.
 */
class AVCHandler {
public:
    explicit AVCHandler(Protocols::AVC::IAVCDiscovery* discovery);
    ~AVCHandler() = default;

    /**
     * @brief Get array of all discovered AV/C units
     *
     * Returns serialized AV/C unit data through IOUserClientMethodArguments.
     *
     * @param args IOUserClientMethodArguments with structureOutput
     * @return kIOReturnSuccess on success, error code otherwise
     */
    kern_return_t GetAVCUnits(IOUserClientMethodArguments* args);

    /**
     * @brief Get capabilities for a specific subunit
     *
     * @param args IOUserClientMethodArguments
     *        - scalarInput[0]: Unit GUID (high 32 bits)
     *        - scalarInput[1]: Unit GUID (low 32 bits)
     *        - scalarInput[2]: Subunit Type
     *        - scalarInput[3]: Subunit ID
     *        - structureOutput: Capabilities data
     * @return kIOReturnSuccess on success
     */
    kern_return_t GetSubunitCapabilities(IOUserClientMethodArguments* args);

    // Helper for testing: Serialize music capabilities to wire format
    // Static and public to allow unit testing without full AVCHandler/AVCDiscovery setup
    static kern_return_t SerializeMusicCapabilities(
        const ASFW::Protocols::AVC::Music::MusicSubunitCapabilities& caps,
        const std::vector<ASFW::Protocols::AVC::Music::MusicSubunit::PlugInfo>& plugs,
        const std::vector<ASFW::Protocols::AVC::Music::MusicSubunit::MusicPlugChannel>& channels,
        IOUserClientMethodArguments* args
    );

    // Helper for testing/UI: serialize Audio-subunit discovery data using the
    // same wire shape the Swift capabilities view already understands.
    static kern_return_t SerializeAudioCapabilities(
        const ASFW::Protocols::AVC::Audio::AudioSubunit& audioSubunit,
        IOUserClientMethodArguments* args
    );

    /**
     * @brief Get raw descriptor data for a specific subunit
     *
     * @param args IOUserClientMethodArguments
     *        - scalarInput[0]: Unit GUID (high 32 bits)
     *        - scalarInput[1]: Unit GUID (low 32 bits)
     *        - scalarInput[2]: Subunit Type
     *        - scalarInput[3]: Subunit ID
     *        - structureOutput: Raw descriptor data
     * @return kIOReturnSuccess on success
     */
    kern_return_t GetSubunitDescriptor(IOUserClientMethodArguments* args);

    /**
     * @brief Submit a raw FCP command asynchronously
     *
     * @param args IOUserClientMethodArguments
     *        - scalarInput[0]: Unit GUID (high 32 bits)
     *        - scalarInput[1]: Unit GUID (low 32 bits)
     *        - structureInput: Raw FCP command bytes (3-512 bytes)
     *        - scalarOutput[0]: Request ID for GetRawFCPCommandResult
     * @return kIOReturnSuccess on successful submission
     */
    kern_return_t SendRawFCPCommand(IOUserClientMethodArguments* args);

    /**
     * @brief Fetch completion/result of a submitted raw FCP command
     *
     * @param args IOUserClientMethodArguments
     *        - scalarInput[0]: Request ID returned by SendRawFCPCommand
     *        - structureOutput: Raw FCP response bytes (if complete/success)
     * @return kIOReturnSuccess when response is available,
     *         kIOReturnNotReady while still pending
     */
    kern_return_t GetRawFCPCommandResult(IOUserClientMethodArguments* args);

    /**
     * @brief Re-scan all AV/C units
     *
     * Triggers re-initialization of all discovered AV/C units.
     *
     * @param args IOUserClientMethodArguments (unused)
     * @return kIOReturnSuccess
     */
    kern_return_t ReScanAVCUnits(IOUserClientMethodArguments* args);

private:
    Protocols::AVC::IAVCDiscovery* discovery_;
};

} // namespace ASFW::UserClient
