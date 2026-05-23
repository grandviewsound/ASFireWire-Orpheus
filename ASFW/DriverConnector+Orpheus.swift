import Foundation

extension ASFWDriverConnector {
    static let orpheusVendorID: UInt32 = 0x001198
    static let orpheusModelID: UInt32 = 0x010048

    // MARK: - Device Selection

    func getOrpheusUnitGUIDs() -> [UInt64] {
        return (getAVCUnits() ?? [])
            .filter { isOrpheusUnit($0) }
            .map { $0.guid }
    }

    func getFirstOrpheusUnitGUID() -> UInt64? {
        return getOrpheusUnitGUIDs().first
    }

    func isOrpheusUnit(_ unit: AVCUnitInfo) -> Bool {
        return unit.vendorID == Self.orpheusVendorID && unit.modelID == Self.orpheusModelID
    }

    // MARK: - Read Full State (Bulk Read per channel)

    func refreshOrpheusState(guid: UInt64, timeoutMs: UInt32 = 15_000) -> OrpheusStateSnapshot? {
        var snapshot = OrpheusStateSnapshot()
        var anySuccess = false

        for ch: UInt8 in 0..<8 {
            if let state = getOrpheusChannelState(guid: guid, channel: ch, timeoutMs: timeoutMs) {
                snapshot.channels[Int(ch)] = state
                anySuccess = true
            }
        }

        guard anySuccess else { return nil }
        snapshot.updatedAt = Date()
        return snapshot
    }

    func getOrpheusChannelState(guid: UInt64, channel: UInt8, timeoutMs: UInt32 = 15_000) -> OrpheusAnalogChannelState? {
        let frame = OrpheusVendorCodec.buildBulkReadFrame(channel: channel)
        guard let response = sendRawFCPCommand(guid: guid, frame: frame, timeoutMs: timeoutMs) else {
            return nil
        }
        return OrpheusVendorCodec.parseBulkResponse(response, channel: channel)
    }

    // MARK: - Read-Only Control Panel Diagnostics

    func refreshOrpheusDiagnostics(guid: UInt64, timeoutMs: UInt32 = 5_000) -> OrpheusDiagnosticsSnapshot {
        var snapshot = OrpheusDiagnosticsSnapshot()
        snapshot.updatedAt = Date()

        if let device = getOrpheusDeviceBulkState(guid: guid, timeoutMs: timeoutMs) {
            snapshot.device = device
        } else {
            snapshot.errors.append("0xBF unit bulk state read failed")
        }

        if let digital = getOrpheusDigitalDiagnosticState(guid: guid, timeoutMs: timeoutMs) {
            snapshot.digital = digital
        } else {
            snapshot.errors.append("0xDF digital bulk state read failed")
        }

        if let version = getOrpheusDeviceVersionState(guid: guid, timeoutMs: timeoutMs) {
            snapshot.deviceVersion = version
        } else {
            snapshot.errors.append("0xB6 hardware version read failed")
        }

        if let noAdat = getOrpheusSignalSource(guid: guid, useAdatPlug: false, timeoutMs: timeoutMs) {
            snapshot.signalSourceNoAdat = noAdat
        } else {
            snapshot.errors.append("SignalSource plug 0x07 read failed")
        }

        if let adat = getOrpheusSignalSource(guid: guid, useAdatPlug: true, timeoutMs: timeoutMs) {
            snapshot.signalSourceAdat = adat
        } else {
            snapshot.errors.append("SignalSource plug 0x08 read failed")
        }

        for channel: UInt8 in 0..<8 {
            if let analog = getOrpheusAnalogDiagnosticState(guid: guid, channel: channel, timeoutMs: timeoutMs) {
                snapshot.analogChannels.append(analog)
            } else {
                snapshot.errors.append("0xCF analog channel \(channel + 1) read failed")
            }
        }

        for output: UInt8 in 0..<6 {
            if let mix = getOrpheusMixOutputState(guid: guid, output: output, timeoutMs: timeoutMs) {
                snapshot.mixOutputs.append(mix)
            } else {
                snapshot.errors.append("0xEF \(OrpheusVendorCodec.mixOutputLabels[Int(output)]) mixer read failed")
            }
        }

        if let meters = refreshOrpheusMeters(guid: guid,
                                             deviceVersion: snapshot.deviceVersion,
                                             timeoutMs: timeoutMs) {
            snapshot.meters = meters
        } else {
            snapshot.errors.append("0xFFC7:00600420 level block read failed")
        }

        return snapshot
    }

    func getOrpheusDeviceBulkState(guid: UInt64, timeoutMs: UInt32 = 5_000) -> OrpheusDeviceBulkState? {
        let frame = OrpheusVendorCodec.buildDeviceFrame(
            isStatus: true,
            commandByte: OrpheusDeviceCommand.bulkDevice.rawValue,
            value: nil)
        guard let response = sendRawFCPCommand(guid: guid, frame: frame, timeoutMs: timeoutMs) else {
            return nil
        }
        return OrpheusVendorCodec.parseDeviceBulkResponse(response)
    }

    func getOrpheusDigitalDiagnosticState(guid: UInt64, timeoutMs: UInt32 = 5_000) -> OrpheusDigitalBulkState? {
        let frame = OrpheusVendorCodec.buildDeviceFrame(
            isStatus: true,
            commandByte: OrpheusDigitalCommand.bulkDigital.rawValue,
            value: nil)
        guard let response = sendRawFCPCommand(guid: guid, frame: frame, timeoutMs: timeoutMs) else {
            return nil
        }
        return OrpheusVendorCodec.parseDigitalDiagnosticResponse(response)
    }

    func getOrpheusDeviceVersionState(guid: UInt64, timeoutMs: UInt32 = 5_000) -> OrpheusDeviceVersionState? {
        let frame = OrpheusVendorCodec.buildDeviceFrame(
            isStatus: true,
            commandByte: OrpheusDeviceCommand.version.rawValue,
            value: nil)
        guard let response = sendRawFCPCommand(guid: guid, frame: frame, timeoutMs: timeoutMs) else {
            return nil
        }
        return OrpheusVendorCodec.parseDeviceVersionResponse(response)
    }

    func refreshOrpheusMeters(guid: UInt64,
                              deviceVersion: OrpheusDeviceVersionState? = nil,
                              timeoutMs: UInt32 = 5_000) -> OrpheusMeterSnapshot? {
        guard let unit = getAVCUnits()?.first(where: { $0.guid == guid }) else {
            return nil
        }

        let useNewLayout = deviceVersion?.usesNewMeterLayout ?? true
        let readLength = useNewLayout
            ? OrpheusVendorCodec.meterReadLengthNew
            : OrpheusVendorCodec.meterReadLengthLegacy

        guard let response = orpheusSyncAsyncBlockRead(
            destinationID: unit.nodeID,
            addressHigh: OrpheusVendorCodec.meterAddressHigh,
            addressLow: OrpheusVendorCodec.meterAddressLow,
            length: readLength,
            timeoutMs: timeoutMs) else {
            return nil
        }

        return OrpheusVendorCodec.parseMeterLevelsResponse(response, usesNewLayout: useNewLayout)
    }

    func getOrpheusSignalSource(guid: UInt64,
                                useAdatPlug: Bool,
                                timeoutMs: UInt32 = 5_000) -> OrpheusSignalSourceState? {
        let frame = OrpheusVendorCodec.buildSignalSourceReadFrame(useAdatPlug: useAdatPlug)
        guard let response = sendRawFCPCommand(guid: guid, frame: frame, timeoutMs: timeoutMs) else {
            return nil
        }
        return OrpheusVendorCodec.parseSignalSourceResponse(response, useAdatPlug: useAdatPlug)
    }

    func setOrpheusSignalSource(guid: UInt64,
                                source: OrpheusSyncSource,
                                useAdatPlug: Bool,
                                timeoutMs: UInt32 = 15_000) -> Bool {
        guard let frame = OrpheusVendorCodec.buildSignalSourceControlFrame(source: source,
                                                                           useAdatPlug: useAdatPlug),
              let response = sendRawFCPCommand(guid: guid, frame: frame, timeoutMs: timeoutMs) else {
            return false
        }
        return OrpheusVendorCodec.isAccepted(response)
    }

    func getOrpheusAnalogDiagnosticState(guid: UInt64,
                                         channel: UInt8,
                                         timeoutMs: UInt32 = 5_000) -> OrpheusAnalogChannelDiagnostic? {
        let frame = OrpheusVendorCodec.buildBulkReadFrame(channel: channel)
        guard let response = sendRawFCPCommand(guid: guid, frame: frame, timeoutMs: timeoutMs) else {
            return nil
        }
        return OrpheusVendorCodec.parseAnalogDiagnosticResponse(response, channel: channel)
    }

    func getOrpheusMixOutputState(guid: UInt64,
                                  output: UInt8,
                                  timeoutMs: UInt32 = 5_000) -> OrpheusMixOutputState? {
        let frame = OrpheusVendorCodec.buildMixOutputReadFrame(output: output)
        guard let response = sendRawFCPCommand(guid: guid, frame: frame, timeoutMs: timeoutMs) else {
            return nil
        }
        return OrpheusVendorCodec.parseMixOutputResponse(response, output: output)
    }

    // MARK: - Boolean Controls

    func setOrpheusPhantom(guid: UInt64, channel: Int, enabled: Bool, timeoutMs: UInt32 = 15_000) -> Bool {
        guard channel >= 0 && channel <= 3 else { return false }
        return orpheusSendBoolControl(.phantom, channel: UInt8(channel), value: enabled, guid: guid, timeoutMs: timeoutMs)
    }

    func setOrpheusOverkiller(guid: UInt64, channel: Int, enabled: Bool, timeoutMs: UInt32 = 15_000) -> Bool {
        guard channel >= 0 && channel <= 7 else { return false }
        return orpheusSendBoolControl(.overkiller, channel: UInt8(channel), value: enabled, guid: guid, timeoutMs: timeoutMs)
    }

    func setOrpheusPhase(guid: UInt64, channel: Int, enabled: Bool, timeoutMs: UInt32 = 15_000) -> Bool {
        guard channel >= 0 && channel <= 7 else { return false }
        return orpheusSendBoolControl(.phase, channel: UInt8(channel), value: enabled, guid: guid, timeoutMs: timeoutMs)
    }

    func setOrpheusMidSide(guid: UInt64, channel: Int, enabled: Bool, timeoutMs: UInt32 = 15_000) -> Bool {
        guard channel >= 0 && channel <= 7 else { return false }
        return orpheusSendBoolControl(.midSide, channel: UInt8(channel), value: enabled, guid: guid, timeoutMs: timeoutMs)
    }

    func setOrpheusLineInLevel(guid: UInt64, channel: Int, plus4dBu: Bool, timeoutMs: UInt32 = 15_000) -> Bool {
        guard channel >= 0 && channel <= 7 else { return false }
        return orpheusSendBoolControl(.lineInLevel, channel: UInt8(channel), value: plus4dBu, guid: guid, timeoutMs: timeoutMs)
    }

    func setOrpheusLineOutLevel(guid: UInt64, channel: Int, plus4dBu: Bool, timeoutMs: UInt32 = 15_000) -> Bool {
        guard channel >= 0 && channel <= 7 else { return false }
        return orpheusSendBoolControl(.lineOutLevel, channel: UInt8(channel), value: plus4dBu, guid: guid, timeoutMs: timeoutMs)
    }

    // MARK: - Value Controls

    func setOrpheusFilter(guid: UInt64, channel: Int, mode: OrpheusFilterMode, timeoutMs: UInt32 = 15_000) -> Bool {
        guard channel >= 0 && channel <= 3 else { return false }
        return orpheusSendValueControl(.filter, channel: UInt8(channel), value: mode.rawValue, guid: guid, timeoutMs: timeoutMs)
    }

    /// Mic preamp gain for channels 0..3. Continuous slider in the original
    /// Prism Control Panel — value byte width per protocol memory file is
    /// `gain1` in the 0xCF bulk read (typical observed values 0..60).
    func setOrpheusMicGain(guid: UInt64, channel: Int, value: UInt8, timeoutMs: UInt32 = 15_000) -> Bool {
        guard channel >= 0 && channel <= 3 else { return false }
        return orpheusSendValueControl(.micGain, channel: UInt8(channel), value: value, guid: guid, timeoutMs: timeoutMs)
    }

    /// Instrument-input gain (channels 0..1 only — the inst-capable inputs).
    /// Maps to the panel's `onInstSlider:` IBAction → `SetAnalog(ch, 4, val)` →
    /// opcode 0xC8.
    func setOrpheusInstGain(guid: UInt64, channel: Int, value: UInt8, timeoutMs: UInt32 = 15_000) -> Bool {
        guard channel >= 0 && channel <= 1 else { return false }
        return orpheusSendValueControl(.instGain, channel: UInt8(channel), value: value, guid: guid, timeoutMs: timeoutMs)
    }

    /// Headphone mix preset (global, not per-channel). Maps to the panel's
    /// `onHeadphoneSelect:` IBAction → `SetAnalog(0, 12, preset)` → opcode 0xCA.
    /// Preset values 0..5 per the panel's HapiButton tags.
    func setOrpheusHeadphoneMix(guid: UInt64, preset: UInt8, timeoutMs: UInt32 = 15_000) -> Bool {
        guard preset <= 5 else { return false }
        // Headphone is a 1-channel "global" call — panel passes ch=0.
        return orpheusSendValueControl(.headphoneMix, channel: 0, value: preset, guid: guid, timeoutMs: timeoutMs)
    }

    // MARK: - Device Settings (meter mode, sample rate, sync source)

    func getOrpheusMeterMode(guid: UInt64, timeoutMs: UInt32 = 15_000) -> UInt8? {
        let frame = OrpheusVendorCodec.buildDeviceFrame(
            isStatus: true, commandByte: OrpheusDeviceCommand.meters.rawValue, value: nil)
        guard let response = sendRawFCPCommand(guid: guid, frame: frame, timeoutMs: timeoutMs) else {
            return nil
        }
        return OrpheusVendorCodec.parseDeviceStatusValue(response)
    }

    func setOrpheusMeterMode(guid: UInt64, mode: UInt8, timeoutMs: UInt32 = 15_000) -> Bool {
        let frame = OrpheusVendorCodec.buildDeviceFrame(
            isStatus: false, commandByte: OrpheusDeviceCommand.meters.rawValue, value: mode)
        guard let response = sendRawFCPCommand(guid: guid, frame: frame, timeoutMs: timeoutMs) else {
            return false
        }
        return OrpheusVendorCodec.isAccepted(response)
    }

    func getOrpheusDigitalSettings(guid: UInt64, timeoutMs: UInt32 = 15_000) -> OrpheusDeviceSettings? {
        let frame = OrpheusVendorCodec.buildDeviceFrame(
            isStatus: true, commandByte: OrpheusDigitalCommand.bulkDigital.rawValue, value: nil)
        guard let response = sendRawFCPCommand(guid: guid, frame: frame, timeoutMs: timeoutMs) else {
            return nil
        }
        return OrpheusVendorCodec.parseBulkDigitalResponse(response)
    }

    func setOrpheusSampleRate(guid: UInt64, rate: UInt8, timeoutMs: UInt32 = 15_000) -> Bool {
        let frame = OrpheusVendorCodec.buildDeviceFrame(
            isStatus: false, commandByte: OrpheusDigitalCommand.sampleRate.rawValue, value: rate)
        guard let response = sendRawFCPCommand(guid: guid, frame: frame, timeoutMs: timeoutMs) else {
            return false
        }
        return OrpheusVendorCodec.isAccepted(response)
    }

    func setOrpheusSyncSource(guid: UInt64, source: UInt8, timeoutMs: UInt32 = 15_000) -> Bool {
        guard let syncSource = OrpheusSyncSource(rawValue: source) else {
            return false
        }
        return setOrpheusSignalSource(guid: guid,
                                      source: syncSource,
                                      useAdatPlug: syncSource == .adat,
                                      timeoutMs: timeoutMs)
    }

    // MARK: - Private Helpers

    private func orpheusSendBoolControl(_ command: OrpheusAnalogCommand, channel: UInt8, value: Bool,
                                        guid: UInt64, timeoutMs: UInt32) -> Bool {
        let frame = OrpheusVendorCodec.buildControlBool(command: command, channel: channel, value: value)
        guard let response = sendRawFCPCommand(guid: guid, frame: frame, timeoutMs: timeoutMs) else {
            return false
        }
        return OrpheusVendorCodec.isAccepted(response)
    }

    private func orpheusSendValueControl(_ command: OrpheusAnalogCommand, channel: UInt8, value: UInt8,
                                         guid: UInt64, timeoutMs: UInt32) -> Bool {
        let frame = OrpheusVendorCodec.buildFrame(isStatus: false, command: command, channel: channel, value: value)
        guard let response = sendRawFCPCommand(guid: guid, frame: frame, timeoutMs: timeoutMs) else {
            return false
        }
        return OrpheusVendorCodec.isAccepted(response)
    }

    private func orpheusSyncAsyncBlockRead(destinationID: UInt16,
                                           addressHigh: UInt16,
                                           addressLow: UInt32,
                                           length: UInt32,
                                           timeoutMs: UInt32) -> Data? {
        guard let handle = asyncBlockRead(
            destinationID: destinationID,
            addressHigh: addressHigh,
            addressLow: addressLow,
            length: length) else {
            return nil
        }

        let timeout = Date().addingTimeInterval(Double(timeoutMs) / 1_000.0)
        while Date() < timeout {
            if let result = getTransactionResult(handle: handle,
                                                 initialPayloadCapacity: Int(length) + 128) {
                guard result.status == 0 && result.responseCode == 0 else {
                    log(String(format: "Orpheus level read failed status=0x%08X rCode=0x%02X",
                               result.status,
                               result.responseCode),
                        level: .warning)
                    return nil
                }
                return result.payload
            }
            Thread.sleep(forTimeInterval: 0.025)
        }

        log(String(format: "Orpheus level read timed out waiting for result (handle=0x%04X)", handle),
            level: .warning)
        return nil
    }
}
