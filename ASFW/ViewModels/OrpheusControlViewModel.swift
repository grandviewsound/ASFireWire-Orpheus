import Foundation
import Combine
import os

private let orpheusDiagnosticsLogger = Logger(subsystem: Bundle.main.bundleIdentifier ?? "ASFW",
                                              category: "OrpheusDiagnostics")

final class OrpheusControlViewModel: ObservableObject {
    @Published var isConnected: Bool = false
    @Published var isLoading: Bool = false
    @Published var isMeterLoading: Bool = false
    @Published var errorMessage: String?

    @Published var orpheusGUID: UInt64?
    @Published var state: OrpheusStateSnapshot = OrpheusStateSnapshot()
    @Published var deviceSettings: OrpheusDeviceSettings = OrpheusDeviceSettings()
    @Published var diagnostics: OrpheusDiagnosticsSnapshot?
    @Published var isSettingSyncSource: Bool = false
    @Published var lastRefreshTime: Date?
    @Published var activeToneLabel: String?
    @Published var toneError: String?
    @Published var toneDeviceName: String?

    /// Panel-wide FP Meters Input/Output setting (mirrors the panel's global combo /
    /// DeviceManager::FpMeters). Persisted to OrpheusGlobals.xml; loaded in init.
    @Published var globalFpMeters: OrpheusFpMeterGlobal = .output

    private let connector: ASFWDriverConnector
    private var cancellables = Set<AnyCancellable>()
    private var lastMeterLogTime: Date?
    private let tonePlayer = DeviceTonePlayer()

    init(connector: ASFWDriverConnector) {
        self.connector = connector
        loadGlobalFpMeters()

        connector.$isConnected
            .receive(on: DispatchQueue.main)
            .sink { [weak self] connected in
                guard let self else { return }
                self.isConnected = connected
                if connected {
                    self.refresh()
                } else {
                    self.orpheusGUID = nil
                    self.errorMessage = "Driver not connected"
                }
            }
            .store(in: &cancellables)

        isConnected = connector.isConnected
    }

    deinit {
        tonePlayer.stop()
    }

    // MARK: - Channel Labels

    static let channelNames: [String] = [
        "AI 1", "AI 2", "AI 3", "AI 4",
        "AI 5", "AI 6", "AI 7", "AI 8"
    ]

    static let channelDescriptions: [String] = [
        "Mic/Line/Inst", "Mic/Line/Inst",
        "Mic/Line", "Mic/Line",
        "Line", "Line", "Line", "Line"
    ]

    // MARK: - Refresh

    func refresh() {
        guard connector.isConnected else {
            errorMessage = "Driver not connected"
            return
        }

        isLoading = true
        errorMessage = nil

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self else { return }

            guard let guid = self.connector.getFirstOrpheusUnitGUID() else {
                DispatchQueue.main.async {
                    self.isLoading = false
                    self.orpheusGUID = nil
                    self.errorMessage = "No Prism Sound Orpheus found"
                }
                return
            }

            let diagnostics = self.connector.refreshOrpheusDiagnostics(guid: guid)
            self.runBlockReadSelfTest(guid: guid)

            DispatchQueue.main.async {
                self.isLoading = false
                self.orpheusGUID = guid
                self.diagnostics = diagnostics

                var stateSnapshot = OrpheusStateSnapshot()
                for analog in diagnostics.analogChannels {
                    stateSnapshot.channels[Int(analog.index)] = analog.state
                }
                if !diagnostics.analogChannels.isEmpty {
                    stateSnapshot.updatedAt = diagnostics.updatedAt
                    self.state = stateSnapshot
                }

                var settings = self.deviceSettings
                if let device = diagnostics.device {
                    settings.meterMode = device.metersMode
                }
                if let digital = diagnostics.digital {
                    settings.sampleRate = digital.sampleRateCode
                    settings.syncSource = digital.syncSource
                    settings.bitDepth = digital.bitDepth
                    settings.digitalInputType = digital.inputType
                    settings.channelStatus = digital.channelStatus
                }
                if let sync = diagnostics.bestSignalSource {
                    settings.syncSource = sync.decodedSync
                }
                self.deviceSettings = settings

                self.lastRefreshTime = Date()
                self.errorMessage = diagnostics.errors.count > 12 ? "Most Orpheus reads failed" : nil
                self.logDiagnosticsSnapshot(diagnostics, reason: "refresh")
            }
        }
    }

    /// One-shot transport check: read a known-nonzero address (Config ROM bus-info
    /// block) through the SAME async block-read path the meter read uses, alongside
    /// the meter block. If Config ROM returns real bytes but the meter block is
    /// all-zero, the block-read transport is fine and the meter problem is device-side;
    /// if Config ROM ALSO comes back zero, the block-read path itself is broken.
    /// Value-independent — no reliance on any remembered/changeable value.
    private func runBlockReadSelfTest(guid: UInt64) {
        func dump(_ data: Data?) -> String {
            guard let data else { return "nil (read failed)" }
            let nonZero = data.reduce(0) { $0 + ($1 != 0 ? 1 : 0) }
            let hex = data.prefix(20).map { String(format: "%02x", $0) }.joined()
            return "len=\(data.count) nonZero=\(nonZero) bytes[0..19]=\(hex)"
        }

        // Config ROM bus-info block @ 0xFFFF:F0000400 — static, always nonzero (quadlet 1 = "1394").
        let rom = connector.orpheusDiagnosticBlockRead(
            guid: guid, addressHigh: 0xFFFF, addressLow: 0xF000_0400, length: 20)
        // The meter level block, read through the identical path.
        let meters = connector.orpheusDiagnosticBlockRead(
            guid: guid,
            addressHigh: OrpheusVendorCodec.meterAddressHigh,
            addressLow: OrpheusVendorCodec.meterAddressLow,
            length: OrpheusVendorCodec.meterReadLengthNew)

        orpheusDiagnosticsLogger.info("OrpheusBlockReadSelfTest configROM[0xFFFF:F0000400] \(dump(rom), privacy: .public)")
        orpheusDiagnosticsLogger.info("OrpheusBlockReadSelfTest meterBlock[0xFFC7:00600420] \(dump(meters), privacy: .public)")
    }

    func refreshMeters() {
        guard connector.isConnected else {
            errorMessage = "Driver not connected"
            return
        }
        guard let guid = orpheusGUID else { return }
        guard !isMeterLoading else { return }

        isMeterLoading = true
        let version = diagnostics?.deviceVersion

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self else { return }
            let meters = self.connector.refreshOrpheusMeters(guid: guid, deviceVersion: version)

            DispatchQueue.main.async {
                self.isMeterLoading = false
                if let meters {
                    var diagnostics = self.diagnostics ?? OrpheusDiagnosticsSnapshot()
                    diagnostics.meters = meters
                    diagnostics.updatedAt = Date()
                    self.diagnostics = diagnostics
                    self.lastRefreshTime = Date()
                    self.logDiagnosticsSnapshot(diagnostics, reason: "meters")
                } else {
                    self.errorMessage = "Meter read failed"
                    orpheusDiagnosticsLogger.error("OrpheusMeters[meters] read failed guid=\(String(format: "0x%016llX", guid), privacy: .public) connectorError=\(self.connector.lastError ?? "none", privacy: .public)")
                }
            }
        }
    }

    // MARK: - Tone Probe

    var isTonePlaying: Bool {
        tonePlayer.isRunning
    }

    func toggleTone(channels: [Int], label: String) {
        if activeToneLabel == label {
            stopTone()
        } else {
            playTone(channels: channels, label: label)
        }
    }

    func playTone(channels: [Int], label: String) {
        guard let device = findOrpheusAudioDevice() else {
            toneError = "ASFW Orpheus Core Audio device not found"
            activeToneLabel = nil
            return
        }

        do {
            try tonePlayer.start(device: device,
                                 channels: channels,
                                 frequency: 1_000,
                                 amplitude: 0.20)
            activeToneLabel = label
            toneDeviceName = device.name
            toneError = nil
            orpheusDiagnosticsLogger.info("OrpheusTone start label=\(label, privacy: .public) device=\(device.name, privacy: .public) uid=\(device.uid, privacy: .public) channels=\(channels.map(String.init).joined(separator: ","), privacy: .public)")
        } catch {
            activeToneLabel = nil
            toneError = error.localizedDescription
            orpheusDiagnosticsLogger.error("OrpheusTone start failed label=\(label, privacy: .public) error=\(error.localizedDescription, privacy: .public)")
        }
    }

    func stopTone() {
        tonePlayer.stop()
        if let activeToneLabel {
            orpheusDiagnosticsLogger.info("OrpheusTone stop label=\(activeToneLabel, privacy: .public)")
        }
        activeToneLabel = nil
    }

    // MARK: - Refresh Single Channel

    private func refreshChannel(_ channel: Int) {
        guard let guid = orpheusGUID else { return }

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self else { return }
            if let ch = self.connector.getOrpheusChannelState(guid: guid, channel: UInt8(channel)) {
                DispatchQueue.main.async {
                    self.state.channels[channel] = ch
                    self.state.updatedAt = Date()
                }
            }
        }
    }

    // MARK: - Boolean Setters

    func setPhantom(channel: Int, enabled: Bool) {
        guard let guid = orpheusGUID else { return }
        state.channels[channel].phantom = enabled

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self else { return }
            let ok = self.connector.setOrpheusPhantom(guid: guid, channel: channel, enabled: enabled)
            if !ok {
                self.refreshChannel(channel)
                DispatchQueue.main.async { self.errorMessage = "Failed to set phantom power" }
            }
        }
    }

    func setOverkiller(channel: Int, enabled: Bool) {
        guard let guid = orpheusGUID else { return }
        state.channels[channel].overkiller = enabled

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self else { return }
            let ok = self.connector.setOrpheusOverkiller(guid: guid, channel: channel, enabled: enabled)
            if !ok {
                self.refreshChannel(channel)
                DispatchQueue.main.async { self.errorMessage = "Failed to set overkiller" }
            }
        }
    }

    func setPhase(channel: Int, enabled: Bool) {
        guard let guid = orpheusGUID else { return }
        state.channels[channel].phase = enabled

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self else { return }
            let ok = self.connector.setOrpheusPhase(guid: guid, channel: channel, enabled: enabled)
            if !ok {
                self.refreshChannel(channel)
                DispatchQueue.main.async { self.errorMessage = "Failed to set phase" }
            }
        }
    }

    func setMidSide(channel: Int, enabled: Bool) {
        guard let guid = orpheusGUID else { return }
        state.channels[channel].midSide = enabled

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self else { return }
            let ok = self.connector.setOrpheusMidSide(guid: guid, channel: channel, enabled: enabled)
            if !ok {
                self.refreshChannel(channel)
                DispatchQueue.main.async { self.errorMessage = "Failed to set mid/side" }
            }
        }
    }

    func setLineInLevel(channel: Int, plus4dBu: Bool) {
        guard let guid = orpheusGUID else { return }
        state.channels[channel].lineInLevel = plus4dBu

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self else { return }
            let ok = self.connector.setOrpheusLineInLevel(guid: guid, channel: channel, plus4dBu: plus4dBu)
            if !ok {
                self.refreshChannel(channel)
                DispatchQueue.main.async { self.errorMessage = "Failed to set line in level" }
            }
        }
    }

    func setLineOutLevel(channel: Int, plus4dBu: Bool) {
        guard let guid = orpheusGUID else { return }
        state.channels[channel].lineOutLevel = plus4dBu

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self else { return }
            let ok = self.connector.setOrpheusLineOutLevel(guid: guid, channel: channel, plus4dBu: plus4dBu)
            if !ok {
                self.refreshChannel(channel)
                DispatchQueue.main.async { self.errorMessage = "Failed to set line out level" }
            }
        }
    }

    func setFilter(channel: Int, mode: OrpheusFilterMode) {
        guard let guid = orpheusGUID else { return }
        state.channels[channel].filter = mode.rawValue

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self else { return }
            let ok = self.connector.setOrpheusFilter(guid: guid, channel: channel, mode: mode)
            if !ok {
                self.refreshChannel(channel)
                DispatchQueue.main.async { self.errorMessage = "Failed to set filter" }
            }
        }
    }

    /// Mic preamp gain — continuous value for channels 0..3. Caller clamps to
    /// the slider's range; we forward the raw byte to the device. On failure
    /// we refresh the channel via the 0xCF bulk read so the UI snaps back to
    /// the device's actual current value.
    func setMicGain(channel: Int, value: UInt8) {
        guard (0...3).contains(channel) else { return }
        guard let guid = orpheusGUID else { return }
        state.channels[channel].micGain = value

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self else { return }
            let ok = self.connector.setOrpheusMicGain(guid: guid, channel: channel, value: value)
            if !ok {
                self.refreshChannel(channel)
                DispatchQueue.main.async { self.errorMessage = "Failed to set mic gain" }
            }
        }
    }

    // MARK: - Device Settings Setters

    func setMeterMode(_ mode: UInt8) {
        guard let guid = orpheusGUID else { return }
        let oldValue = deviceSettings.meterMode
        deviceSettings.meterMode = mode

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self else { return }
            let ok = self.connector.setOrpheusMeterMode(guid: guid, mode: mode)
            if !ok {
                DispatchQueue.main.async {
                    self.deviceSettings.meterMode = oldValue
                    self.errorMessage = "Failed to set meter mode"
                }
            }
        }
    }

    // MARK: - Front Panel Meters (local + global)

    /// Current per-device FP Meters selection, decoded from the device byte.
    var localFpMeters: OrpheusFpMeterLocal {
        OrpheusFpMeterLocal.from(deviceByte: deviceSettings.meterMode)
    }

    /// True when this unit is set to follow the panel-wide global meter mode.
    var isFollowingGlobalFpMeters: Bool {
        (deviceSettings.meterMode & 0b10) != 0
    }

    /// Set the per-device FP Meters selection (Input / Output / Follow Global).
    /// Mirrors onFpMetersLocal: — Follow Global resolves to `global | 2`.
    func setLocalFpMeters(_ selection: OrpheusFpMeterLocal) {
        setMeterMode(selection.deviceByte(global: globalFpMeters))
    }

    /// Set the panel-wide global Input/Output meter mode. Mirrors
    /// DeviceManager::SetFpMeters: persist it, then re-push to the connected unit
    /// if (and only if) it is in Follow-Global mode.
    func setGlobalFpMeters(_ global: OrpheusFpMeterGlobal) {
        guard global != globalFpMeters || isFollowingGlobalFpMeters else {
            globalFpMeters = global
            return
        }
        globalFpMeters = global
        persistGlobalFpMeters(global)
        if isFollowingGlobalFpMeters {
            setMeterMode(global.followGlobalDeviceByte)
        }
    }

    // MARK: OrpheusGlobals.xml persistence
    //
    // The original panel wrote `./OrpheusGlobals.xml` (a relative-path bug). We use a
    // stable Application Support location. Schema: <Devices><FpMeters>N</FpMeters></Devices>.

    private var orpheusGlobalsURL: URL? {
        FileManager.default.urls(for: .applicationSupportDirectory, in: .userDomainMask).first?
            .appendingPathComponent("ASFW/OrpheusGlobals.xml")
    }

    private func loadGlobalFpMeters() {
        guard let url = orpheusGlobalsURL,
              let xml = try? String(contentsOf: url, encoding: .utf8),
              let value = OrpheusControlViewModel.parseFpMeters(from: xml),
              let global = OrpheusFpMeterGlobal(rawValue: value) else {
            return
        }
        globalFpMeters = global
    }

    private func persistGlobalFpMeters(_ global: OrpheusFpMeterGlobal) {
        guard let url = orpheusGlobalsURL else { return }
        let xml = "<Devices><FpMeters>\(global.rawValue)</FpMeters></Devices>\n"
        do {
            try FileManager.default.createDirectory(at: url.deletingLastPathComponent(),
                                                    withIntermediateDirectories: true)
            try xml.write(to: url, atomically: true, encoding: .utf8)
        } catch {
            orpheusDiagnosticsLogger.error("Failed to persist OrpheusGlobals.xml: \(error.localizedDescription, privacy: .public)")
        }
    }

    /// Extract N from `<FpMeters>N</FpMeters>`.
    static func parseFpMeters(from xml: String) -> UInt8? {
        guard let open = xml.range(of: "<FpMeters>"),
              let close = xml.range(of: "</FpMeters>", range: open.upperBound..<xml.endIndex) else {
            return nil
        }
        return UInt8(xml[open.upperBound..<close.lowerBound].trimmingCharacters(in: .whitespacesAndNewlines))
    }

    func setSyncSource(_ source: UInt8) {
        guard let syncSource = OrpheusSyncSource(rawValue: source) else {
            errorMessage = "Unsupported sync source"
            return
        }
        setSyncSource(syncSource)
    }

    func setSyncSource(_ source: OrpheusSyncSource) {
        guard let guid = orpheusGUID else { return }
        let oldValue = deviceSettings.syncSource
        deviceSettings.syncSource = source.rawValue
        isSettingSyncSource = true
        let useAdatPlug = shouldUseAdatSignalSourcePlug(for: source)

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self else { return }
            let ok = self.connector.setOrpheusSignalSource(guid: guid,
                                                           source: source,
                                                           useAdatPlug: useAdatPlug)
            DispatchQueue.main.async {
                self.isSettingSyncSource = false
                if ok {
                    self.refresh()
                } else {
                    self.deviceSettings.syncSource = oldValue
                    self.errorMessage = "Failed to set sync source"
                }
            }
        }
    }

    func setSampleRate(_ rate: UInt8) {
        guard let guid = orpheusGUID else { return }
        let oldValue = deviceSettings.sampleRate
        deviceSettings.sampleRate = rate

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self else { return }
            let ok = self.connector.setOrpheusSampleRate(guid: guid, rate: rate)
            if !ok {
                DispatchQueue.main.async {
                    self.deviceSettings.sampleRate = oldValue
                    self.errorMessage = "Failed to set sample rate"
                }
            }
        }
    }

    private func logDiagnosticsSnapshot(_ diagnostics: OrpheusDiagnosticsSnapshot, reason: String) {
        if reason == "meters" {
            let now = Date()
            if let lastMeterLogTime, now.timeIntervalSince(lastMeterLogTime) < 1.0 {
                return
            }
            lastMeterLogTime = now
        }

        let guidLabel = orpheusGUID.map { String(format: "0x%016llX", $0) } ?? "unknown"
        let source = diagnostics.device?.outputSourceName ?? "unread"
        let sync = diagnostics.bestSignalSource.map {
            String(format: "%@/%@ plug=0x%02X raw4=0x%02X raw5=0x%02X",
                   $0.syncName, $0.riskLabel, $0.plug, $0.responseByte4, $0.responseByte5)
        } ?? "unread"
        let version = diagnostics.deviceVersion?.displayName ?? "unread"
        let dawMeters = meterSummary(diagnostics.meters, group: .dawFeeds)
        let outputMeters = meterSummary(diagnostics.meters, group: .physicalOutputs)
        let analogInputMeters = meterSummary(diagnostics.meters, group: .analogInputs)
        let digitalInputMeters = meterSummary(diagnostics.meters, group: .digitalInputs)
        // Full unparsed level block + a nonzero-byte count, so we can tell an all-zero
        // buffer apart from a parse-offset bug, and see whether ANY region is live.
        let rawAll = diagnostics.meters?.raw ?? Data()
        let rawHex = rawAll.map { String(format: "%02x", $0) }.joined()
        let rawNonZero = rawAll.reduce(0) { $0 + ($1 != 0 ? 1 : 0) }
        // The +200 trailer carries master volume. We KNOW masterVol from the 0xBF bulk
        // read, so if the trailer matches but the level slots are zero, the block read
        // lands correctly and the device's metering DSP is simply dormant for our session.
        let trailer = diagnostics.meters?.trailer.map {
            String(format: "masterVol=%d analogType=%@ async=%d unlock=%d",
                   $0.masterVolume, "\($0.analogType)", $0.digitalAsync ? 1 : 0, $0.digitalUnlock ? 1 : 0)
        } ?? "none"
        let connectorError = diagnostics.errors.isEmpty ? "none" : (connector.lastError ?? "none")

        orpheusDiagnosticsLogger.info("OrpheusDiagnostics[\(reason, privacy: .public)] guid=\(guidLabel, privacy: .public) version=\(version, privacy: .public) source=\(source, privacy: .public) sync=\(sync, privacy: .public) errors=\(diagnostics.errors.count, privacy: .public) connectorError=\(connectorError, privacy: .public)")
        orpheusDiagnosticsLogger.info("OrpheusMeters[\(reason, privacy: .public)] inputs=\(analogInputMeters, privacy: .public) digIn=\(digitalInputMeters, privacy: .public) daw=\(dawMeters, privacy: .public) outputs=\(outputMeters, privacy: .public)")
        orpheusDiagnosticsLogger.info("OrpheusMetersRaw[\(reason, privacy: .public)] nonZeroBytes=\(rawNonZero, privacy: .public) trailer=\(trailer, privacy: .public) raw=\(rawHex, privacy: .public)")
        if !diagnostics.errors.isEmpty {
            let errorSummary = diagnostics.errors.prefix(8).joined(separator: " | ")
            orpheusDiagnosticsLogger.info("OrpheusDiagnosticsErrors[\(reason, privacy: .public)] \(errorSummary, privacy: .public)")
        }

        for path in diagnostics.playbackPaths {
            let route = "out=\(path.label) mode=\(path.modeLabel) muted=\(path.outputMuted ? 1 : 0) daw=\(path.dawRawPairLabel) physical=\(path.outputRawPairLabel) mixerDAW=\(path.mixerDawLabel) probe=\(path.probeLabel)"
            orpheusDiagnosticsLogger.info("OrpheusPath[\(reason, privacy: .public)] \(route, privacy: .public)")
        }
    }

    private func meterSummary(_ meters: OrpheusMeterSnapshot?,
                              group: OrpheusMeterLevelState.Group) -> String {
        guard let meters else { return "unread" }
        return meters.levels(in: group)
            .map { "\($0.label)=\($0.rawLabel)" }
            .joined(separator: ",")
    }

    private func findOrpheusAudioDevice() -> AudioWrapperDevice? {
        let guidLabel = orpheusGUID.map { String(format: "%016llX", $0).lowercased() }
        let devices = AudioSystem.shared.devices

        return devices.first { device in
            let haystack = "\(device.uid) \(device.name) \(device.modelUID)".lowercased()
            if let guidLabel, haystack.contains(guidLabel) {
                return true
            }
            return haystack.contains("orpheus")
                || haystack.contains("asfwaudiodevice")
                || haystack.contains("asfw")
        } ?? devices.first { device in
            device.transportType == .fireWire && device.outputChannelCount >= 12
        }
    }

    private func shouldUseAdatSignalSourcePlug(for source: OrpheusSyncSource) -> Bool {
        if source == .adat {
            return true
        }
        if diagnostics?.hasAdatInput == true {
            return true
        }
        return diagnostics?.bestSignalSource?.plug == 0x08
    }
}
