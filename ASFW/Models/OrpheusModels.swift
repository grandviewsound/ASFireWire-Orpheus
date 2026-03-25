import Foundation

// MARK: - Orpheus Wire Constants

struct OrpheusVendorWireConstants {
    static let oui: [UInt8] = [0x00, 0x11, 0x98]
    static let subunitByte: UInt8 = 0x08
    static let opcodeVendorDependent: UInt8 = 0x00
    static let ctypeControl: UInt8 = 0x00
    static let ctypeStatus: UInt8 = 0x01
    static let responseAccepted: UInt8 = 0x09
    static let responseStable: UInt8 = 0x0C
    static let frameLength = 15
}

// MARK: - Analog Command Bytes (SetAnalog / GetAnalog)

enum OrpheusAnalogCommand: UInt8 {
    case overkiller     = 0xC1
    case phantom        = 0xC2
    case filter         = 0xC3
    case phase          = 0xC4
    case midSide        = 0xC5
    case lineInLevel    = 0xC6
    case micGain        = 0xC7
    case impedance      = 0xC8
    case lineOutLevel   = 0xC9
    case headphoneMix   = 0xCA
    case bulkAnalog     = 0xCF
}

// MARK: - Digital Command Bytes (SetDigital)

enum OrpheusDigitalCommand: UInt8 {
    case sampleRate     = 0xD0
    case bitDepth       = 0xD2
    case syncSource     = 0xD3
    case inputType      = 0xD4
    case channelStatus  = 0xD7
    case bulkDigital    = 0xDF
}

// MARK: - Device Command Bytes (Set / Get, non-channel)

enum OrpheusDeviceCommand: UInt8 {
    case meters          = 0xB0
    case source          = 0xB1
    case wordclock       = 0xB2
    case adat            = 0xB3
    case meterBrightness = 0xB7
    case bulkDevice      = 0xBF
}

// MARK: - Meter Display Mode

enum OrpheusMeterMode: UInt8, CaseIterable, Identifiable {
    case input  = 0
    case output = 1

    var id: UInt8 { rawValue }

    var displayName: String {
        switch self {
        case .input:  return "Input"
        case .output: return "Output"
        }
    }
}

// MARK: - Sync Source

enum OrpheusSyncSource: UInt8, CaseIterable, Identifiable {
    case `internal` = 0
    case spdif      = 1
    case wordClock  = 2
    case adat       = 3

    var id: UInt8 { rawValue }

    var displayName: String {
        switch self {
        case .internal:  return "Internal"
        case .spdif:     return "S/PDIF"
        case .wordClock: return "Word Clock"
        case .adat:      return "ADAT"
        }
    }
}

// MARK: - Sample Rate (wire values — provisional mapping, verify with device)

enum OrpheusSampleRate: UInt8, CaseIterable, Identifiable {
    case rate32000  = 0
    case rate44100  = 1
    case rate48000  = 2
    case rate88200  = 3
    case rate96000  = 4
    case rate176400 = 5
    case rate192000 = 6

    var id: UInt8 { rawValue }

    var displayName: String {
        switch self {
        case .rate32000:  return "32 kHz"
        case .rate44100:  return "44.1 kHz"
        case .rate48000:  return "48 kHz"
        case .rate88200:  return "88.2 kHz"
        case .rate96000:  return "96 kHz"
        case .rate176400: return "176.4 kHz"
        case .rate192000: return "192 kHz"
        }
    }
}

// MARK: - Device-Level Settings

struct OrpheusDeviceSettings: Equatable {
    var meterMode: UInt8 = 0
    var sampleRate: UInt8 = 0
    var syncSource: UInt8 = 0
    var bitDepth: UInt8 = 0
    var digitalInputType: UInt8 = 0
    var channelStatus: UInt8 = 0
}

// MARK: - Analog Channel State

struct OrpheusAnalogChannelState: Equatable {
    var lineOutLevel: Bool = true        // true = +4dBu, false = -10dBV
    var lineInLevel: Bool = true         // true = +4dBu, false = -10dBV
    var overkiller: Bool = false
    var midSide: Bool = false
    var phantom: Bool = false            // 48V, channels 0-3 only
    var phase: Bool = false

    var filter: UInt8 = 0               // 0=off, 1=HP, 2=RIAA; channels 0-3 only
    var micGain: UInt8 = 0              // channels 0-3 only
    var impedance: UInt8 = 0            // channels 0-1 only
    var type: UInt8 = 0                 // input type identifier

    /// Parse the packed "All" boolean byte from bulk read response byte 9
    mutating func parseAllByte(_ byte: UInt8) {
        lineOutLevel = (byte & 0x01) != 0
        lineInLevel  = (byte & 0x02) != 0
        overkiller   = (byte & 0x04) != 0
        midSide      = (byte & 0x08) != 0
        phantom      = (byte & 0x20) != 0
        phase        = (byte & 0x40) != 0
    }
}

enum OrpheusFilterMode: UInt8, CaseIterable, Identifiable {
    case off  = 0
    case hp   = 1
    case riaa = 2

    var id: UInt8 { rawValue }

    var displayName: String {
        switch self {
        case .off:  return "Off"
        case .hp:   return "HPF"
        case .riaa: return "RIAA"
        }
    }
}

// MARK: - Device State Snapshot

struct OrpheusStateSnapshot: Equatable {
    var channels: [OrpheusAnalogChannelState] = Array(repeating: OrpheusAnalogChannelState(), count: 8)
    var updatedAt: Date?
}

// MARK: - Frame Builder

struct OrpheusVendorCodec {
    /// Build a CONTROL frame to set a boolean analog parameter
    static func buildControlBool(command: OrpheusAnalogCommand, channel: UInt8, value: Bool) -> Data {
        return buildFrame(isStatus: false, command: command, channel: channel, value: value ? 0x01 : 0x00)
    }

    /// Build a STATUS frame for bulk analog read
    static func buildBulkReadFrame(channel: UInt8) -> Data {
        return buildFrame(isStatus: true, command: .bulkAnalog, channel: channel, value: nil)
    }

    /// Build a raw vendor-dependent AVC frame (always 15 bytes)
    static func buildFrame(isStatus: Bool, command: OrpheusAnalogCommand, channel: UInt8, value: UInt8?) -> Data {
        var frame = Data(count: OrpheusVendorWireConstants.frameLength)
        frame[0] = isStatus ? OrpheusVendorWireConstants.ctypeStatus : OrpheusVendorWireConstants.ctypeControl
        frame[1] = OrpheusVendorWireConstants.subunitByte
        frame[2] = OrpheusVendorWireConstants.opcodeVendorDependent
        frame[3] = OrpheusVendorWireConstants.oui[0]
        frame[4] = OrpheusVendorWireConstants.oui[1]
        frame[5] = OrpheusVendorWireConstants.oui[2]
        frame[6] = command.rawValue
        frame[7] = channel
        if let v = value {
            frame[8] = v
        } else {
            frame[8] = 0xFF
        }
        // bytes 9-14 padded with 0xFF
        for i in 9..<OrpheusVendorWireConstants.frameLength {
            frame[i] = 0xFF
        }
        return frame
    }

    /// Parse a bulk read (0xCF STATUS) response into channel state
    static func parseBulkResponse(_ response: Data, channel: UInt8) -> OrpheusAnalogChannelState? {
        guard response.count >= 12,
              response[0] == OrpheusVendorWireConstants.responseStable else {
            return nil
        }

        var state = OrpheusAnalogChannelState()
        state.type = response[8]
        state.parseAllByte(response[9])

        if channel <= 3 && response.count >= 12 {
            state.filter = response[10]
            state.micGain = response[11]
        }
        if channel <= 1 && response.count >= 13 {
            state.impedance = response[12]
        }

        return state
    }

    /// Check if a CONTROL response indicates success
    static func isAccepted(_ response: Data) -> Bool {
        return response.count >= 1 && response[0] == OrpheusVendorWireConstants.responseAccepted
    }

    // MARK: - Device / Digital Frames (no channel byte)

    /// Build a vendor-dependent frame for device/digital commands (no channel byte)
    static func buildDeviceFrame(isStatus: Bool, commandByte: UInt8, value: UInt8?) -> Data {
        var frame = Data(count: OrpheusVendorWireConstants.frameLength)
        frame[0] = isStatus ? OrpheusVendorWireConstants.ctypeStatus : OrpheusVendorWireConstants.ctypeControl
        frame[1] = OrpheusVendorWireConstants.subunitByte
        frame[2] = OrpheusVendorWireConstants.opcodeVendorDependent
        frame[3] = OrpheusVendorWireConstants.oui[0]
        frame[4] = OrpheusVendorWireConstants.oui[1]
        frame[5] = OrpheusVendorWireConstants.oui[2]
        frame[6] = commandByte
        if let v = value {
            frame[7] = v
        } else {
            frame[7] = 0xFF
        }
        for i in 8..<OrpheusVendorWireConstants.frameLength {
            frame[i] = 0xFF
        }
        return frame
    }

    /// Parse a STATUS response for a device/digital command (value at byte 7)
    static func parseDeviceStatusValue(_ response: Data) -> UInt8? {
        guard response.count >= 8,
              response[0] == OrpheusVendorWireConstants.responseStable else {
            return nil
        }
        return response[7]
    }

    /// Parse bulk digital (0xDF) STATUS response into device settings
    static func parseBulkDigitalResponse(_ response: Data) -> OrpheusDeviceSettings? {
        guard response.count >= 12,
              response[0] == OrpheusVendorWireConstants.responseStable else {
            return nil
        }
        var settings = OrpheusDeviceSettings()
        settings.digitalInputType = response[7]
        settings.channelStatus = response[8]
        settings.bitDepth = response[9]
        settings.sampleRate = response[10]
        settings.syncSource = response[11]
        return settings
    }
}
