import Foundation
import Combine

final class OrpheusControlViewModel: ObservableObject {
    @Published var isConnected: Bool = false
    @Published var isLoading: Bool = false
    @Published var errorMessage: String?

    @Published var orpheusGUID: UInt64?
    @Published var state: OrpheusStateSnapshot = OrpheusStateSnapshot()
    @Published var deviceSettings: OrpheusDeviceSettings = OrpheusDeviceSettings()
    @Published var lastRefreshTime: Date?

    private let connector: ASFWDriverConnector
    private var cancellables = Set<AnyCancellable>()

    init(connector: ASFWDriverConnector) {
        self.connector = connector

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

            let snapshot = self.connector.refreshOrpheusState(guid: guid)

            // Read device-level settings (meter mode, digital config)
            let meterMode = self.connector.getOrpheusMeterMode(guid: guid)
            let digitalSettings = self.connector.getOrpheusDigitalSettings(guid: guid)

            DispatchQueue.main.async {
                self.isLoading = false
                self.orpheusGUID = guid

                guard let snapshot else {
                    self.errorMessage = "Failed to read Orpheus state"
                    return
                }

                self.state = snapshot

                // Merge device settings from both reads
                var settings = digitalSettings ?? self.deviceSettings
                if let meter = meterMode {
                    settings.meterMode = meter
                }
                self.deviceSettings = settings

                self.lastRefreshTime = Date()
                self.errorMessage = nil
            }
        }
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

    func setSyncSource(_ source: UInt8) {
        guard let guid = orpheusGUID else { return }
        let oldValue = deviceSettings.syncSource
        deviceSettings.syncSource = source

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self else { return }
            let ok = self.connector.setOrpheusSyncSource(guid: guid, source: source)
            if !ok {
                DispatchQueue.main.async {
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
}
