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
        let frame = OrpheusVendorCodec.buildDeviceFrame(
            isStatus: false, commandByte: OrpheusDigitalCommand.syncSource.rawValue, value: source)
        guard let response = sendRawFCPCommand(guid: guid, frame: frame, timeoutMs: timeoutMs) else {
            return false
        }
        return OrpheusVendorCodec.isAccepted(response)
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
}
