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
    /// Instrument-input gain for the inst-capable channels (0..1). The Prism
    /// panel labels this "Inst gain"; the original Swift port had it as
    /// `impedance` which is wrong per the analysis RE (`onInstSlider:` →
    /// `Device::SetAnalog(ch, 4, val)` → opcode 0xC8).
    case instGain       = 0xC8
    case lineOutLevel   = 0xC9
    case headphoneMix   = 0xCA
    case bulkAnalog     = 0xCF
}

enum OrpheusMixCommand: UInt8 {
    case bulkMix = 0xEF
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
    case version         = 0xB6
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

// MARK: - Front Panel Meters (FP Meters)
//
// The device's meter setting — opcode 0xB0, mirrored in the 0xBF bulk byte-11 high
// nibble (`metersMode`) — is a 2-bit value:
//   bit 0 → 0 = Input, 1 = Output   (what the front-panel LEDs display)
//   bit 1 → "Follow Global": this unit tracks the panel-wide global Input/Output setting
// so all units in a multi-unit rig switch together (manual: hardware_met.htm).
// Confirmed from the Prism panel's behavior: onFpMetersLocal: (Follow Global → global|2),
// onFpMetersGlobal: → DeviceManager::SetFpMeters (re-pushes global|2 to every Follow-Global
// unit).

/// Per-device FP Meters selection (Unit Settings tab): Input / Output / Follow Global.
enum OrpheusFpMeterLocal: UInt8, CaseIterable, Identifiable {
    case input        = 0
    case output       = 1
    case followGlobal = 2

    var id: UInt8 { rawValue }

    var displayName: String {
        switch self {
        case .input:        return "Input"
        case .output:       return "Output"
        case .followGlobal: return "Follow Global"
        }
    }

    /// Decode the raw device byte (0xB0 value / `metersMode` nibble) into a selection.
    static func from(deviceByte: UInt8) -> OrpheusFpMeterLocal {
        if (deviceByte & 0b10) != 0 { return .followGlobal }
        return (deviceByte & 0b01) != 0 ? .output : .input
    }

    /// Encode into the device byte to send via 0xB0. The global setting is only
    /// consulted for `.followGlobal`, mirroring onFpMetersLocal: (`global | 2`).
    func deviceByte(global: OrpheusFpMeterGlobal) -> UInt8 {
        switch self {
        case .input:        return 0
        case .output:       return 1
        case .followGlobal: return (global.rawValue & 0b01) | 0b10
        }
    }
}

/// Panel-wide FP Meters global Input/Output setting; persisted in OrpheusGlobals.xml.
enum OrpheusFpMeterGlobal: UInt8, CaseIterable, Identifiable {
    case input  = 0
    case output = 1

    var id: UInt8 { rawValue }

    var displayName: String {
        switch self {
        case .input:  return "Input"
        case .output: return "Output"
        }
    }

    /// Device byte pushed to a Follow-Global unit when the global changes.
    var followGlobalDeviceByte: UInt8 { (rawValue & 0b01) | 0b10 }
}

// MARK: - Sync Source

enum OrpheusSyncSource: UInt8, CaseIterable, Identifiable {
    case local      = 0
    case freeRun    = 1
    case wordClock  = 2
    case spdif      = 3
    case adat       = 4
    case slave      = 5

    var id: UInt8 { rawValue }

    var displayName: String {
        switch self {
        case .local:     return "Local"
        case .freeRun:   return "Free-run"
        case .wordClock: return "Word Clock"
        case .spdif:     return "S/PDIF"
        case .adat:      return "ADAT"
        case .slave:     return "Slave"
        }
    }

    var isExternalClock: Bool {
        switch self {
        case .wordClock, .spdif, .adat:
            return true
        case .local, .freeRun, .slave:
            return false
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

    /// Parse the packed "All" boolean byte from bulk read response byte 9.
    /// analysis: Analog::GetAll() packs input level at bit 0 and output level at bit 1.
    mutating func parseAllByte(_ byte: UInt8) {
        lineInLevel  = (byte & 0x01) != 0
        lineOutLevel = (byte & 0x02) != 0
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

// MARK: - Read-Only Diagnostic Snapshot

struct OrpheusDeviceBulkState: Equatable {
    var masterVolume: Int16
    var masterEnabled: Int16
    var masterMute: Bool
    var masterLock: Bool
    var bit11_2: Bool
    var metersMode: UInt8
    var outputSource: UInt8
    var wordclock: UInt8
    var adatMode: UInt8
    var headphoneMix: UInt8
    var meterBrightness: UInt8
    var raw: Data

    var outputSourceName: String {
        outputSource == 1 ? "FireWire/DAW" : String(format: "Unknown 0x%X", outputSource)
    }

    var headphoneMixName: String {
        switch headphoneMix {
        case 0: return "AO1/2"
        case 1: return "AO3/4"
        case 2: return "AO5/6"
        case 3: return "AO7/8"
        case 4: return "DO1/2"
        case 5: return "Headphone mix"
        case 0xF: return "No bus"
        default: return String(format: "0x%X", headphoneMix)
        }
    }

    var metersModeName: String {
        switch metersMode {
        case 0: return "Input"
        case 1: return "Output"
        default: return String(format: "0x%X", metersMode)
        }
    }

    var masterEnabledMask: String {
        String(format: "0x%04X", UInt16(bitPattern: masterEnabled))
    }

    var hasAdatInput: Bool {
        (2...4).contains(adatMode)
    }
}

struct OrpheusDigitalBulkState: Equatable {
    var inputType: UInt8
    var channelStatus: UInt8
    var bitDepth: UInt8
    var sampleRateCode: UInt8
    var syncSource: UInt8
    var unlocked: Bool
    var asynchronous: Bool
    var raw: Data
}

struct OrpheusDeviceVersionState: Equatable {
    var major: UInt8
    var minor: UInt8
    var raw: Data

    /// analysis: Device::ReadLevels() uses the 204-byte level block path when version > 1.02.
    var usesNewMeterLayout: Bool {
        major > 1 || (major == 1 && minor > 2)
    }

    var displayName: String {
        String(format: "%d.%02d", Int(major), Int(minor))
    }
}

struct OrpheusSignalSourceState: Equatable {
    var plug: UInt8
    var responseByte4: UInt8
    var responseByte5: UInt8
    var decodedSync: UInt8
    var raw: Data

    var syncName: String {
        switch decodedSync {
        case 0: return "Local"
        case 1: return "Free-run"
        case 2: return "Wordclock"
        case 3: return "S/PDIF"
        case 4: return "ADAT"
        case 5: return "Slave"
        default: return "Unknown"
        }
    }

    var riskLabel: String {
        switch decodedSync {
        case 2, 3, 4: return "External"
        case 0, 1, 5: return "Internal/host-safe"
        default: return "Unknown"
        }
    }

    var syncSource: OrpheusSyncSource? {
        OrpheusSyncSource(rawValue: decodedSync)
    }
}

struct OrpheusMeterLevelState: Equatable, Identifiable {
    enum Group: String, Equatable {
        case analogInputs
        case digitalInputs
        case physicalOutputs
        case dawFeeds
        case adatSend
        case adatReturn

        var displayName: String {
            switch self {
            case .analogInputs: return "Analog Inputs"
            case .digitalInputs: return "Digital Inputs"
            case .physicalOutputs: return "Outputs"
            case .dawFeeds: return "DAW Feeds"
            case .adatSend: return "ADAT Send"
            case .adatReturn: return "ADAT Return"
            }
        }
    }

    var index: UInt8
    var label: String
    var group: Group
    var raw: UInt16

    var id: UInt8 { index }

    var rawLabel: String {
        String(format: "0x%04X", raw)
    }

    var normalized: Double {
        min(max(Double(raw) / 65_535.0, 0), 1)
    }
}

/// 4-byte trailer at the end of the new-firmware (204-byte) meter buffer.
/// Decoded from `Orpheus::Device::ReadLevels` — the panel uses this to detect
/// state changes (master vol, mute, lock, analog input type, digital sync)
/// without a separate poll. Posts CFNotifications when values change.
struct OrpheusMeterTrailer: Equatable {
    /// Master volume — 16-bit big-endian, signed
    var masterVolume: Int16
    /// Per-channel analog input type (0..3), 2 bits each, for channels 0..3
    var analogType: [UInt8]
    /// Digital `SetAsync` flag (byte 3 bit 1 == 0x02)
    var digitalAsync: Bool
    /// Digital `SetUnlok` flag (byte 3 bit 2 == 0x04)
    var digitalUnlock: Bool
}

struct OrpheusMeterSnapshot: Equatable {
    var addressHigh: UInt16
    var addressLow: UInt32
    var readLength: UInt32
    var usesNewLayout: Bool
    var levels: [OrpheusMeterLevelState]
    var raw: Data
    /// Present when `usesNewLayout` is true and the response is ≥ 204 bytes.
    var trailer: OrpheusMeterTrailer?

    func levels(in group: OrpheusMeterLevelState.Group) -> [OrpheusMeterLevelState] {
        levels.filter { $0.group == group }
    }

    func level(at index: Int) -> OrpheusMeterLevelState? {
        guard levels.indices.contains(index) else { return nil }
        return levels[index]
    }

    var anyDawFeedActive: Bool {
        levels(in: .dawFeeds).contains { $0.raw != 0 }
    }

    var anyOutputActive: Bool {
        levels(in: .physicalOutputs).contains { $0.raw != 0 }
    }

    var addressLabel: String {
        String(format: "0x%04X:%08X", addressHigh, addressLow)
    }
}

struct OrpheusAnalogChannelDiagnostic: Equatable {
    var index: UInt8
    var state: OrpheusAnalogChannelState
    var raw: Data
}

struct OrpheusMixInputState: Equatable, Identifiable {
    var index: UInt8
    var label: String
    var gain: Int16
    var muted: Bool
    var usesBalance: Bool
    var panOrBalance: Int8

    var id: UInt8 { index }

    var gainLabel: String {
        gain == Int16.min ? "-inf" : "\(gain)"
    }

    var panLabel: String {
        let prefix = usesBalance ? "bal" : "pan"
        return "\(prefix) \(panOrBalance)"
    }
}

struct OrpheusMixOutputState: Equatable, Identifiable {
    var index: UInt8
    var label: String
    var gain: Int16
    var muted: Bool
    var soloMask: UInt16
    var defeated: Bool
    var inputs: [OrpheusMixInputState]
    var raw: Data

    var id: UInt8 { index }

    var modeLabel: String {
        defeated ? "Direct" : "Mixer"
    }

    var soloMaskLabel: String {
        String(format: "0x%04X", soloMask)
    }
}

struct OrpheusPlaybackPathState: Equatable, Identifiable {
    var outputIndex: UInt8
    var label: String
    var directMode: Bool
    var outputMuted: Bool
    var outputGain: Int16
    var dawLeft: OrpheusMeterLevelState?
    var dawRight: OrpheusMeterLevelState?
    var outputLeft: OrpheusMeterLevelState?
    var outputRight: OrpheusMeterLevelState?
    var mixerDawLeft: OrpheusMixInputState?
    var mixerDawRight: OrpheusMixInputState?

    var id: UInt8 { outputIndex }

    var modeLabel: String {
        directMode ? "Direct" : "Mixer"
    }

    var dawRawPairLabel: String {
        rawPairLabel(dawLeft, dawRight)
    }

    var outputRawPairLabel: String {
        rawPairLabel(outputLeft, outputRight)
    }

    var dawActive: Bool {
        (dawLeft?.raw ?? 0) != 0 || (dawRight?.raw ?? 0) != 0
    }

    var outputActive: Bool {
        (outputLeft?.raw ?? 0) != 0 || (outputRight?.raw ?? 0) != 0
    }

    var mixerDawOpen: Bool {
        guard let left = mixerDawLeft, let right = mixerDawRight else { return false }
        return !left.muted && !right.muted && (left.gain != Int16.min || right.gain != Int16.min)
    }

    var mixerDawLabel: String {
        guard let left = mixerDawLeft, let right = mixerDawRight else { return "unread" }
        return "\(left.gainLabel)/\(right.gainLabel) \(left.muted || right.muted ? "muted" : "open")"
    }

    var probeLabel: String {
        if outputMuted {
            return "Output muted"
        }
        if outputActive {
            return "Output active"
        }
        if !dawActive {
            return "No DAW feed"
        }
        if directMode {
            return "Direct gap"
        }
        if !mixerDawOpen {
            return "Mixer DAW closed"
        }
        return "Mixer gap"
    }

    private func rawPairLabel(_ left: OrpheusMeterLevelState?, _ right: OrpheusMeterLevelState?) -> String {
        "\(left?.rawLabel ?? "----") / \(right?.rawLabel ?? "----")"
    }
}

struct OrpheusDiagnosticsSnapshot {
    var device: OrpheusDeviceBulkState?
    var digital: OrpheusDigitalBulkState?
    var deviceVersion: OrpheusDeviceVersionState?
    var signalSourceNoAdat: OrpheusSignalSourceState?
    var signalSourceAdat: OrpheusSignalSourceState?
    var meters: OrpheusMeterSnapshot?
    var analogChannels: [OrpheusAnalogChannelDiagnostic] = []
    var mixOutputs: [OrpheusMixOutputState] = []
    var errors: [String] = []
    var updatedAt: Date?

    var bestSignalSource: OrpheusSignalSourceState? {
        if let noAdat = signalSourceNoAdat, noAdat.decodedSync != 0xFF {
            return noAdat
        }
        return signalSourceAdat
    }

    var hasAdatInput: Bool {
        if let device {
            return device.hasAdatInput
        }
        return bestSignalSource?.plug == 0x08
    }

    var playbackPaths: [OrpheusPlaybackPathState] {
        guard let meters else { return [] }
        return mixOutputs.sorted { $0.index < $1.index }.map { output in
            let outputIndex = Int(output.index)
            let meterOutputBase = 10 + outputIndex * 2
            let meterDawBase = 22 + outputIndex * 2
            return OrpheusPlaybackPathState(
                outputIndex: output.index,
                label: output.label,
                directMode: output.defeated,
                outputMuted: output.muted,
                outputGain: output.gain,
                dawLeft: meters.level(at: meterDawBase),
                dawRight: meters.level(at: meterDawBase + 1),
                outputLeft: meters.level(at: meterOutputBase),
                outputRight: meters.level(at: meterOutputBase + 1),
                mixerDawLeft: output.inputs.first { $0.index == 10 },
                mixerDawRight: output.inputs.first { $0.index == 11 }
            )
        }
    }
}

// MARK: - Frame Builder

struct OrpheusVendorCodec {
    static let mixOutputLabels = [
        "Analog 1/2", "Analog 3/4", "Analog 5/6",
        "Analog 7/8", "Digital 1/2", "Headphones"
    ]

    static let mixInputLabels = [
        "Analog 1", "Analog 2", "Analog 3", "Analog 4",
        "Analog 5", "Analog 6", "Analog 7", "Analog 8",
        "Digital L", "Digital R", "DAW L", "DAW R"
    ]

    static let meterAddressHigh: UInt16 = 0xFFC7
    static let meterAddressLow: UInt32 = 0x0060_0420
    static let meterReadLengthNew: UInt32 = 204
    static let meterReadLengthLegacy: UInt32 = 200

    /// analysis: HapiMainView::meterTimerFired maps Device._levels[0...49] to these controls.
    static let meterSlotLabels: [(String, OrpheusMeterLevelState.Group)] = [
        ("AI1", .analogInputs), ("AI2", .analogInputs), ("AI3", .analogInputs), ("AI4", .analogInputs),
        ("AI5", .analogInputs), ("AI6", .analogInputs), ("AI7", .analogInputs), ("AI8", .analogInputs),
        ("DI1", .digitalInputs), ("DI2", .digitalInputs),
        ("AO1", .physicalOutputs), ("AO2", .physicalOutputs), ("AO3", .physicalOutputs), ("AO4", .physicalOutputs),
        ("AO5", .physicalOutputs), ("AO6", .physicalOutputs), ("AO7", .physicalOutputs), ("AO8", .physicalOutputs),
        ("DO1", .physicalOutputs), ("DO2", .physicalOutputs), ("HP L", .physicalOutputs), ("HP R", .physicalOutputs),
        ("DAW AO1", .dawFeeds), ("DAW AO2", .dawFeeds), ("DAW AO3", .dawFeeds), ("DAW AO4", .dawFeeds),
        ("DAW AO5", .dawFeeds), ("DAW AO6", .dawFeeds), ("DAW AO7", .dawFeeds), ("DAW AO8", .dawFeeds),
        ("DAW DO1", .dawFeeds), ("DAW DO2", .dawFeeds), ("DAW HP L", .dawFeeds), ("DAW HP R", .dawFeeds),
        ("ADAT S1", .adatSend), ("ADAT S2", .adatSend), ("ADAT S3", .adatSend), ("ADAT S4", .adatSend),
        ("ADAT S5", .adatSend), ("ADAT S6", .adatSend), ("ADAT S7", .adatSend), ("ADAT S8", .adatSend),
        ("ADAT R1", .adatReturn), ("ADAT R2", .adatReturn), ("ADAT R3", .adatReturn), ("ADAT R4", .adatReturn),
        ("ADAT R5", .adatReturn), ("ADAT R6", .adatReturn), ("ADAT R7", .adatReturn), ("ADAT R8", .adatReturn)
    ]

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

    static func parseAnalogDiagnosticResponse(_ response: Data, channel: UInt8) -> OrpheusAnalogChannelDiagnostic? {
        guard let state = parseBulkResponse(response, channel: channel) else {
            return nil
        }
        return OrpheusAnalogChannelDiagnostic(index: channel, state: state, raw: response)
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

    static func buildMixOutputReadFrame(output: UInt8) -> Data {
        var frame = Data(count: 74)
        frame[0] = OrpheusVendorWireConstants.ctypeStatus
        frame[1] = OrpheusVendorWireConstants.subunitByte
        frame[2] = OrpheusVendorWireConstants.opcodeVendorDependent
        frame[3] = OrpheusVendorWireConstants.oui[0]
        frame[4] = OrpheusVendorWireConstants.oui[1]
        frame[5] = OrpheusVendorWireConstants.oui[2]
        frame[6] = OrpheusMixCommand.bulkMix.rawValue
        frame[7] = output
        return frame
    }

    static func buildSignalSourceReadFrame(useAdatPlug: Bool) -> Data {
        var frame = Data(count: 8)
        frame[0] = OrpheusVendorWireConstants.ctypeStatus
        frame[1] = 0xFF
        frame[2] = 0x1A
        frame[3] = 0x0F
        frame[4] = 0xFF
        frame[5] = 0xFF
        frame[6] = 0x60
        frame[7] = useAdatPlug ? 0x08 : 0x07
        return frame
    }

    /// analysis: `Orpheus::Device::SetSyncAvc(int)` builds this exact AV/C
    /// SignalSource CONTROL frame. The destination plug is 0x07 normally and
    /// 0x08 when the ADAT input mode is active.
    static func buildSignalSourceControlFrame(source: OrpheusSyncSource,
                                              useAdatPlug: Bool) -> Data? {
        let destinationPlug: UInt8 = useAdatPlug ? 0x08 : 0x07
        var frame = Data(count: 8)
        frame[0] = OrpheusVendorWireConstants.ctypeControl
        frame[1] = 0xFF
        frame[2] = 0x1A
        frame[3] = 0x0F
        frame[6] = 0x60
        frame[7] = destinationPlug

        switch source {
        case .local:
            frame[4] = 0x60
            frame[5] = destinationPlug + 1
        case .freeRun:
            frame[4] = 0xFF
            frame[5] = 0x00
        case .wordClock:
            frame[4] = 0xFF
            frame[5] = useAdatPlug ? 0x88 : 0x87
        case .spdif:
            frame[4] = 0xFF
            frame[5] = 0x85
        case .adat:
            guard useAdatPlug else { return nil }
            frame[4] = 0xFF
            frame[5] = 0x86
        case .slave:
            frame[4] = 0xFF
            frame[5] = useAdatPlug ? 0x87 : 0x86
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

    static func parseDeviceBulkResponse(_ response: Data) -> OrpheusDeviceBulkState? {
        guard response.count >= 15,
              response[0] == OrpheusVendorWireConstants.responseStable,
              response[3] == OrpheusVendorWireConstants.oui[0],
              response[4] == OrpheusVendorWireConstants.oui[1],
              response[5] == OrpheusVendorWireConstants.oui[2],
              response[6] == OrpheusDeviceCommand.bulkDevice.rawValue else {
            return nil
        }

        let masterVolume = readInt16BE(response, offset: 7)
        let masterEnabled = readInt16BE(response, offset: 9)
        let pack11 = response[11]
        let pack12 = response[12]
        let pack14 = response[14]

        return OrpheusDeviceBulkState(
            masterVolume: masterVolume,
            masterEnabled: masterEnabled,
            masterMute: (pack11 & 0x01) != 0,
            masterLock: (pack11 & 0x02) != 0,
            bit11_2: (pack11 & 0x04) != 0,
            metersMode: pack11 >> 4,
            outputSource: pack12 & 0x0F,
            wordclock: pack12 >> 4,
            adatMode: response[13],
            headphoneMix: pack14 & 0x0F,
            meterBrightness: pack14 >> 4,
            raw: response
        )
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

    static func parseDigitalDiagnosticResponse(_ response: Data) -> OrpheusDigitalBulkState? {
        guard response.count >= 13,
              response[0] == OrpheusVendorWireConstants.responseStable,
              response[3] == OrpheusVendorWireConstants.oui[0],
              response[4] == OrpheusVendorWireConstants.oui[1],
              response[5] == OrpheusVendorWireConstants.oui[2],
              response[6] == OrpheusDigitalCommand.bulkDigital.rawValue else {
            return nil
        }

        let flags = response[12]
        return OrpheusDigitalBulkState(
            inputType: response[7],
            channelStatus: response[8],
            bitDepth: response[9],
            sampleRateCode: response[10],
            syncSource: response[11],
            unlocked: (flags & 0x01) != 0,
            asynchronous: (flags & 0x02) != 0,
            raw: response
        )
    }

    static func parseDeviceVersionResponse(_ response: Data) -> OrpheusDeviceVersionState? {
        guard response.count >= 9,
              response[0] == OrpheusVendorWireConstants.responseStable,
              response[3] == OrpheusVendorWireConstants.oui[0],
              response[4] == OrpheusVendorWireConstants.oui[1],
              response[5] == OrpheusVendorWireConstants.oui[2],
              response[6] == OrpheusDeviceCommand.version.rawValue else {
            return nil
        }

        return OrpheusDeviceVersionState(major: response[7], minor: response[8], raw: response)
    }

    static func parseMeterLevelsResponse(_ response: Data, usesNewLayout: Bool) -> OrpheusMeterSnapshot? {
        guard response.count >= Int(meterReadLengthLegacy) else {
            return nil
        }

        var levels: [OrpheusMeterLevelState] = []
        let slotCount = min(meterSlotLabels.count, response.count / 4)
        for slot in 0..<slotCount {
            let base = slot * 4
            let valueOffset = (usesNewLayout || slot < 22) ? base + 2 : base
            guard response.count > valueOffset + 1 else { break }

            let raw = readUInt16BE(response, offset: valueOffset)
            let label = meterSlotLabels[slot]
            levels.append(OrpheusMeterLevelState(
                index: UInt8(slot),
                label: label.0,
                group: label.1,
                raw: raw
            ))
        }

        guard levels.count == meterSlotLabels.count else {
            return nil
        }

        // New-firmware (204-byte) meter buffer carries a 4-byte trailer at +200
        // with master vol / analog type / digital sync. Decoded in pass 5 of the
        // panel RE — the panel uses this same trailer for state-change detection.
        var trailer: OrpheusMeterTrailer?
        if usesNewLayout, response.count >= 204 {
            let masterVol = Int16(bitPattern: readUInt16BE(response, offset: 200))
            let typeByte = response[202]
            let syncByte = response[203]
            trailer = OrpheusMeterTrailer(
                masterVolume: masterVol,
                analogType: [
                    typeByte & 0b11,
                    (typeByte >> 2) & 0b11,
                    (typeByte >> 4) & 0b11,
                    (typeByte >> 6) & 0b11
                ],
                digitalAsync: (syncByte & 0x02) != 0,
                digitalUnlock: (syncByte & 0x04) != 0
            )
        }

        return OrpheusMeterSnapshot(
            addressHigh: meterAddressHigh,
            addressLow: meterAddressLow,
            readLength: usesNewLayout ? meterReadLengthNew : meterReadLengthLegacy,
            usesNewLayout: usesNewLayout,
            levels: levels,
            raw: response,
            trailer: trailer
        )
    }

    static func parseMixOutputResponse(_ response: Data, output: UInt8) -> OrpheusMixOutputState? {
        guard output < 6,
              response.count >= 74,
              response[0] == OrpheusVendorWireConstants.responseStable,
              response[3] == OrpheusVendorWireConstants.oui[0],
              response[4] == OrpheusVendorWireConstants.oui[1],
              response[5] == OrpheusVendorWireConstants.oui[2],
              response[6] == OrpheusMixCommand.bulkMix.rawValue,
              response[7] == output else {
            return nil
        }

        var inputs: [OrpheusMixInputState] = []
        for input in 0..<12 {
            let base = 14 + input * 5
            let gain = readInt16BE(response, offset: base)
            let label = mixInputLabels[input]
            inputs.append(OrpheusMixInputState(
                index: UInt8(input),
                label: label,
                gain: gain,
                muted: response[base + 2] != 0,
                usesBalance: response[base + 3] != 0,
                panOrBalance: Int8(bitPattern: response[base + 4])
            ))
        }

        return OrpheusMixOutputState(
            index: output,
            label: mixOutputLabels[Int(output)],
            gain: readInt16BE(response, offset: 8),
            muted: response[10] != 0,
            soloMask: readUInt16BE(response, offset: 11),
            defeated: response[13] != 0,
            inputs: inputs,
            raw: response
        )
    }

    static func parseSignalSourceResponse(_ response: Data, useAdatPlug: Bool) -> OrpheusSignalSourceState? {
        guard response.count >= 8,
              response[0] == OrpheusVendorWireConstants.responseStable else {
            return nil
        }

        let decoded = decodeSignalSource(byte4: response[4], byte5: response[5], useAdatPlug: useAdatPlug)
        return OrpheusSignalSourceState(
            plug: useAdatPlug ? 0x08 : 0x07,
            responseByte4: response[4],
            responseByte5: response[5],
            decodedSync: decoded,
            raw: response
        )
    }

    private static func decodeSignalSource(byte4: UInt8, byte5: UInt8, useAdatPlug: Bool) -> UInt8 {
        if byte4 == 0x60 {
            return 0
        }
        guard byte4 == 0xFF else {
            return 0xFF
        }

        if useAdatPlug {
            switch byte5 {
            case 0x00: return 1
            case 0x85: return 3
            case 0x86: return 4
            case 0x87: return 5
            case 0x88: return 2
            default: return 0xFF
            }
        }

        switch byte5 {
        case 0x00: return 1
        case 0x85: return 3
        case 0x86: return 5
        case 0x87: return 2
        default: return 0xFF
        }
    }

    private static func readUInt16BE(_ data: Data, offset: Int) -> UInt16 {
        // Slice-safe: index relative to startIndex, since a sliced Data does not
        // re-base its indices to 0.
        guard data.count > offset + 1 else { return 0 }
        let b = data.startIndex
        return (UInt16(data[b + offset]) << 8) | UInt16(data[b + offset + 1])
    }

    private static func readInt16BE(_ data: Data, offset: Int) -> Int16 {
        Int16(bitPattern: readUInt16BE(data, offset: offset))
    }
}

extension Data {
    var orpheusHexString: String {
        map { String(format: "%02X", $0) }.joined(separator: " ")
    }
}
