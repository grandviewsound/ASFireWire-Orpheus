import SwiftUI
import Combine

struct OrpheusControlView: View {
    @StateObject private var viewModel: OrpheusControlViewModel
    @State private var selectedMixerOutput: UInt8 = 0
    // 20 Hz to match Prism control panel's NSTimer (pass 5 RE).
    // Meter polling is also the state-change detection mechanism for master vol,
    // mute, lock, analog type and digital sync (trailer fields in the 204-byte
    // meter read). Always poll while the page is visible — the original panel
    // had no on/off toggle.
    private let meterRefreshTimer = Timer.publish(every: 0.05, on: .main, in: .common).autoconnect()
    private let tonePairs: [(label: String, channels: [Int])] = [
        ("1/2", [0, 1]), ("3/4", [2, 3]), ("5/6", [4, 5]),
        ("7/8", [6, 7]), ("9/10", [8, 9]), ("11/12", [10, 11])
    ]

    init(connector: ASFWDriverConnector) {
        _viewModel = StateObject(wrappedValue: OrpheusControlViewModel(connector: connector))
    }

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 18) {
                headerSection

                if !viewModel.isConnected {
                    stateCard(title: "Driver Not Connected",
                              message: "Connect to ASFWDriver to control Orpheus settings.")
                } else if viewModel.orpheusGUID == nil {
                    stateCard(title: "No Prism Sound Orpheus Found",
                              message: "Connect the Orpheus and refresh.")
                } else {
                    diagnosticsDashboard
                }

                if let error = viewModel.errorMessage {
                    Label(error, systemImage: "exclamationmark.triangle.fill")
                        .foregroundStyle(.orange)
                        .font(.callout)
                }
            }
            .padding()
        }
        .navigationTitle("Orpheus")
        .onAppear {
            viewModel.refresh()
        }
        .onReceive(meterRefreshTimer) { _ in
            // Always poll while the Orpheus page is visible. The Prism panel
            // does the same — 20 Hz unconditional. State-change detection on
            // master vol, mute, lock, analog type and digital sync rides on
            // this poll's 4-byte trailer (parsed in the view-model).
            if viewModel.orpheusGUID != nil {
                viewModel.refreshMeters()
            }
        }
    }

    // MARK: - Header

    private var headerSection: some View {
        GroupBox {
            HStack(alignment: .center) {
                VStack(alignment: .leading, spacing: 6) {
                    Text("Prism Sound Orpheus")
                        .font(.headline)

                    if let guid = viewModel.orpheusGUID {
                        Text(String(format: "GUID 0x%016llX", guid))
                            .font(.system(.caption, design: .monospaced))
                            .foregroundStyle(.secondary)
                    } else {
                        Text("No device selected")
                            .foregroundStyle(.secondary)
                    }

                    if let refreshed = viewModel.lastRefreshTime {
                        Text("Updated \(refreshed.formatted(date: .omitted, time: .standard))")
                            .font(.caption2)
                            .foregroundStyle(.secondary)
                    }
                }

                Spacer()

                if viewModel.isLoading {
                    ProgressView()
                        .controlSize(.small)
                }

                if viewModel.isMeterLoading {
                    ProgressView()
                        .controlSize(.small)
                }

                Button {
                    viewModel.refresh()
                } label: {
                    Label("Refresh", systemImage: "arrow.clockwise")
                }
                .disabled(viewModel.isLoading)
            }
        } label: {
            Label("Read-Only Diagnostics", systemImage: "hifispeaker.2.fill")
                .font(.headline)
        }
    }

    // MARK: - Diagnostics Dashboard

    private var diagnosticsDashboard: some View {
        VStack(alignment: .leading, spacing: 18) {
            toneProbeSection

            if let diagnostics = viewModel.diagnostics {
                diagnosticsOverview(diagnostics)
                syncSourceControl(diagnostics)
                meterDiagnostics(diagnostics)
                playbackPathDiagnostics(diagnostics)
                outputSetupDiagnostics(diagnostics)
                mixerDiagnostics(diagnostics)
                inputDiagnostics(diagnostics)
                micChannelsSection
                lineChannelsSection
                rawDiagnostics(diagnostics)

                if !diagnostics.errors.isEmpty {
                    GroupBox {
                        VStack(alignment: .leading, spacing: 6) {
                            ForEach(diagnostics.errors, id: \.self) { error in
                                Label(error, systemImage: "exclamationmark.triangle.fill")
                                    .foregroundStyle(.orange)
                                    .font(.caption)
                            }
                        }
                    } label: {
                        Label("Read Gaps", systemImage: "exclamationmark.triangle")
                            .font(.headline)
                    }
                }
            } else {
                stateCard(title: "Waiting For Readback",
                          message: "Refresh to read the Orpheus vendor state.")
            }
        }
    }

    private var toneProbeSection: some View {
        GroupBox {
            VStack(alignment: .leading, spacing: 12) {
                HStack(spacing: 10) {
                    if let active = viewModel.activeToneLabel {
                        Label(active, systemImage: "waveform")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    } else if let deviceName = viewModel.toneDeviceName {
                        Text(deviceName)
                            .font(.caption)
                            .foregroundStyle(.secondary)
                            .lineLimit(1)
                    }

                    Spacer()

                    Button {
                        viewModel.stopTone()
                    } label: {
                        Label("Stop", systemImage: "stop.fill")
                    }
                    .disabled(!viewModel.isTonePlaying)
                }

                LazyVGrid(columns: [GridItem(.adaptive(minimum: 88), spacing: 8)], spacing: 8) {
                    ForEach(tonePairs, id: \.label) { pair in
                        Button {
                            viewModel.toggleTone(channels: pair.channels, label: "Pair \(pair.label)")
                        } label: {
                            Label(pair.label,
                                  systemImage: viewModel.activeToneLabel == "Pair \(pair.label)" ? "stop.fill" : "play.fill")
                                .frame(maxWidth: .infinity)
                        }
                        .buttonStyle(.bordered)
                        .disabled(viewModel.orpheusGUID == nil)
                    }
                }

                if let toneError = viewModel.toneError {
                    Label(toneError, systemImage: "exclamationmark.triangle.fill")
                        .font(.caption)
                        .foregroundStyle(.orange)
                }
            }
        } label: {
            Label("Tone", systemImage: "waveform")
                .font(.headline)
        }
    }

    private func meterDiagnostics(_ diagnostics: OrpheusDiagnosticsSnapshot) -> some View {
        GroupBox {
            if let meters = diagnostics.meters {
                VStack(alignment: .leading, spacing: 14) {
                    HStack(spacing: 10) {
                        diagnosticMetric("Level Block",
                                         meters.usesNewLayout ? "204-byte" : "200-byte",
                                         raw: meters.addressLabel)
                        diagnosticMetric("DAW Feeds",
                                         meters.anyDawFeedActive ? "Active" : "Silent",
                                         raw: meters.levels(in: .dawFeeds).map(\.rawLabel).joined(separator: " "))
                        diagnosticMetric("Outputs",
                                         meters.anyOutputActive ? "Active" : "Silent",
                                         raw: meters.levels(in: .physicalOutputs).map(\.rawLabel).joined(separator: " "))
                    }

                    meterGroup(.dawFeeds, meters: meters)
                    meterGroup(.physicalOutputs, meters: meters)
                    meterGroup(.analogInputs, meters: meters)
                    meterGroup(.digitalInputs, meters: meters)

                    DisclosureGroup {
                        VStack(alignment: .leading, spacing: 10) {
                            meterGroup(.adatSend, meters: meters)
                            meterGroup(.adatReturn, meters: meters)
                        }
                        .padding(.top, 8)
                    } label: {
                        Text("ADAT")
                            .font(.subheadline.bold())
                    }
                }
            } else {
                Text("No meter readback")
                    .foregroundStyle(.secondary)
            }
        } label: {
            Label("Meters", systemImage: "waveform.path.ecg")
                .font(.headline)
        }
    }

    private func diagnosticsOverview(_ diagnostics: OrpheusDiagnosticsSnapshot) -> some View {
        GroupBox {
            LazyVGrid(columns: [GridItem(.adaptive(minimum: 180), spacing: 12)], spacing: 12) {
                if let device = diagnostics.device {
                    diagnosticMetric("Source", device.outputSourceName, raw: hex(device.outputSource))
                    diagnosticMetric("Master", device.masterMute ? "Muted" : "Open",
                                     raw: "\(device.masterVolume)")
                    diagnosticMetric("Assigned Vol", device.masterEnabledMask,
                                     raw: device.masterLock ? "locked" : "unlocked")
                    diagnosticMetric("Front Meters", device.metersModeName,
                                     raw: "brightness \(device.meterBrightness)")
                    diagnosticMetric("Headphones", device.headphoneMixName,
                                     raw: "ADAT \(device.adatMode)")
                }

                if let version = diagnostics.deviceVersion {
                    diagnosticMetric("Hardware", version.displayName,
                                     raw: version.usesNewMeterLayout ? "new meters" : "legacy meters")
                }

                if let sync = diagnostics.bestSignalSource {
                    diagnosticMetric("Clock", sync.syncName,
                                     raw: "\(sync.riskLabel) plug \(hex(sync.plug))")
                }

                if let digital = diagnostics.digital {
                    diagnosticMetric("Digital In", digital.unlocked ? "Unlocked" : "Locked",
                                     raw: digital.asynchronous ? "async" : "sync")
                    diagnosticMetric("Digital Out", "bits \(digital.bitDepth)",
                                     raw: "rate \(digital.sampleRateCode)")
                }
            }
        } label: {
            Label("Unit", systemImage: "dial.medium")
                .font(.headline)
        }
    }

    private func syncSourceControl(_ diagnostics: OrpheusDiagnosticsSnapshot) -> some View {
        let sync = diagnostics.bestSignalSource
        let selectedSource = sync?.syncSource
            ?? OrpheusSyncSource(rawValue: viewModel.deviceSettings.syncSource)
            ?? .local

        return GroupBox {
            VStack(alignment: .leading, spacing: 12) {
                HStack(spacing: 12) {
                    diagnosticMetric("Current", sync?.syncName ?? "Unread",
                                     raw: sync.map { "\(hex($0.responseByte4)) \(hex($0.responseByte5))" } ?? "--")
                    diagnosticMetric("Clock", sync?.riskLabel ?? "Unknown",
                                     raw: sync.map { "plug \(hex($0.plug))" } ?? "plug --")
                    diagnosticMetric("ADAT Input", diagnostics.hasAdatInput ? "Enabled" : "Not active",
                                     raw: diagnostics.device.map { "mode \($0.adatMode)" } ?? "mode --")

                    if viewModel.isSettingSyncSource {
                        ProgressView()
                            .controlSize(.small)
                    }
                }

                HStack {
                    Picker("Source", selection: Binding(
                        get: { selectedSource },
                        set: { viewModel.setSyncSource($0) }
                    )) {
                        ForEach(OrpheusSyncSource.allCases) { source in
                            Text(source.displayName)
                                .tag(source)
                                .disabled(source == .adat && !diagnostics.hasAdatInput)
                        }
                    }
                    .pickerStyle(.menu)
                    .frame(width: 180, alignment: .leading)
                    .disabled(viewModel.isSettingSyncSource || viewModel.orpheusGUID == nil)

                    statusPill(selectedSource.isExternalClock ? "External" : "Internal/host-safe",
                               color: selectedSource.isExternalClock ? .orange : .green)

                    Spacer()
                }
            }
        } label: {
            Label("Sync Source", systemImage: "clock.badge.checkmark")
                .font(.headline)
        }
    }

    private func playbackPathDiagnostics(_ diagnostics: OrpheusDiagnosticsSnapshot) -> some View {
        GroupBox {
            if diagnostics.playbackPaths.isEmpty {
                Text("No playback path readback")
                    .foregroundStyle(.secondary)
            } else {
                VStack(alignment: .leading, spacing: 8) {
                    ForEach(diagnostics.playbackPaths) { path in
                        playbackPathRow(path)
                        if path.outputIndex != diagnostics.playbackPaths.last?.outputIndex {
                            Divider()
                        }
                    }
                }
            }
        } label: {
            Label("Playback Path Probe", systemImage: "point.topleft.down.curvedto.point.bottomright.up")
                .font(.headline)
        }
    }

    private func playbackPathRow(_ path: OrpheusPlaybackPathState) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack(spacing: 10) {
                Text(path.label)
                    .font(.subheadline.bold())
                    .frame(width: 110, alignment: .leading)

                statusPill(path.modeLabel, color: path.directMode ? .blue : .green)
                    .frame(width: 70, alignment: .leading)

                statusPill(path.probeLabel, color: playbackProbeColor(path))

                Spacer()
            }

            LazyVGrid(columns: [GridItem(.adaptive(minimum: 160), spacing: 10)], spacing: 8) {
                diagnosticMetric("DAW Feed", path.dawActive ? "Active" : "Silent",
                                 raw: path.dawRawPairLabel)
                diagnosticMetric("Output Meter", path.outputActive ? "Active" : "Silent",
                                 raw: path.outputRawPairLabel)
                diagnosticMetric("Output State", path.outputMuted ? "Muted" : "Open",
                                 raw: "gain \(path.outputGain)")
                diagnosticMetric("Mixer DAW", path.directMode ? "Bypassed" : path.mixerDawLabel,
                                 raw: "slots 10/11")
            }
        }
        .padding(.vertical, 6)
    }

    private func outputSetupDiagnostics(_ diagnostics: OrpheusDiagnosticsSnapshot) -> some View {
        GroupBox {
            VStack(alignment: .leading, spacing: 8) {
                ForEach(diagnostics.mixOutputs) { output in
                    HStack(spacing: 12) {
                        Text(output.label)
                            .font(.subheadline.bold())
                            .frame(width: 110, alignment: .leading)

                        statusPill(output.modeLabel, color: output.defeated ? .blue : .green)
                            .frame(width: 70, alignment: .leading)

                        Text("gain \(output.gain)")
                            .font(.system(.caption, design: .monospaced))
                            .frame(width: 80, alignment: .leading)

                        statusPill(output.muted ? "Muted" : "Open",
                                   color: output.muted ? .orange : .secondary)
                            .frame(width: 70, alignment: .leading)

                        Text("solo \(output.soloMaskLabel)")
                            .font(.system(.caption, design: .monospaced))
                            .foregroundStyle(.secondary)

                        Spacer()
                    }
                    .padding(.vertical, 4)

                    if output.index != diagnostics.mixOutputs.last?.index {
                        Divider()
                    }
                }
            }
        } label: {
            Label("Outputs", systemImage: "speaker.wave.2.fill")
                .font(.headline)
        }
    }

    private func mixerDiagnostics(_ diagnostics: OrpheusDiagnosticsSnapshot) -> some View {
        GroupBox {
            VStack(alignment: .leading, spacing: 12) {
                if diagnostics.mixOutputs.isEmpty {
                    Text("No mixer readback")
                        .foregroundStyle(.secondary)
                } else {
                    Picker("Mixer", selection: $selectedMixerOutput) {
                        ForEach(diagnostics.mixOutputs) { output in
                            Text(output.label).tag(output.index)
                        }
                    }
                    .pickerStyle(.segmented)

                    if let output = selectedMixerOutputState(diagnostics) {
                        HStack(spacing: 12) {
                            Text(output.label)
                                .font(.subheadline.bold())
                            statusPill(output.modeLabel, color: output.defeated ? .blue : .green)
                            statusPill(output.muted ? "Muted" : "Open",
                                       color: output.muted ? .orange : .secondary)
                            Text("gain \(output.gain)")
                                .font(.system(.caption, design: .monospaced))
                            Spacer()
                        }

                        VStack(alignment: .leading, spacing: 6) {
                            ForEach(output.inputs) { input in
                                mixerInputRow(input)
                            }
                        }
                    }
                }
            }
        } label: {
            Label("Mixer", systemImage: "slider.vertical.3")
                .font(.headline)
        }
    }

    private func inputDiagnostics(_ diagnostics: OrpheusDiagnosticsSnapshot) -> some View {
        GroupBox {
            LazyVGrid(columns: [GridItem(.adaptive(minimum: 190), spacing: 12)], spacing: 12) {
                ForEach(diagnostics.analogChannels, id: \.index) { channel in
                    let state = channel.state
                    VStack(alignment: .leading, spacing: 8) {
                        HStack {
                            Text("Analog \(channel.index + 1)")
                                .font(.subheadline.bold())
                            Spacer()
                            Text("type \(state.type)")
                                .font(.system(.caption2, design: .monospaced))
                                .foregroundStyle(.secondary)
                        }
                        HStack(spacing: 6) {
                            statusPill(state.lineInLevel ? "In +4" : "In -10", color: .secondary)
                            statusPill(state.lineOutLevel ? "Out +4" : "Out -10", color: .secondary)
                        }
                        HStack(spacing: 6) {
                            if state.phantom { statusPill("48V", color: .orange) }
                            if state.phase { statusPill("Phase", color: .orange) }
                            if state.midSide { statusPill("M/S", color: .orange) }
                            if state.overkiller { statusPill("OVK", color: .orange) }
                        }
                        Text("filter \(state.filter) gain \(state.micGain) imp \(state.impedance)")
                            .font(.system(.caption2, design: .monospaced))
                            .foregroundStyle(.secondary)
                    }
                    .padding(10)
                    .background(Color.secondary.opacity(0.08))
                    .clipShape(RoundedRectangle(cornerRadius: 8))
                }
            }
        } label: {
            Label("Inputs", systemImage: "waveform")
                .font(.headline)
        }
    }

    private func rawDiagnostics(_ diagnostics: OrpheusDiagnosticsSnapshot) -> some View {
        DisclosureGroup {
            VStack(alignment: .leading, spacing: 8) {
                if let device = diagnostics.device {
                    rawLine("0xBF Unit", device.raw)
                }
                if let digital = diagnostics.digital {
                    rawLine("0xDF Digital", digital.raw)
                }
                if let signal = diagnostics.signalSourceNoAdat {
                    rawLine("SignalSource 0x07", signal.raw)
                }
                if let signal = diagnostics.signalSourceAdat {
                    rawLine("SignalSource 0x08", signal.raw)
                }
                ForEach(diagnostics.mixOutputs) { output in
                    rawLine("0xEF \(output.label)", output.raw)
                }
                ForEach(diagnostics.analogChannels, id: \.index) { channel in
                    rawLine("0xCF Analog \(channel.index + 1)", channel.raw)
                }
            }
            .padding(.top, 8)
        } label: {
            Label("Raw Responses", systemImage: "curlybraces")
                .font(.headline)
        }
        .padding(.horizontal, 4)
    }

    private func selectedMixerOutputState(_ diagnostics: OrpheusDiagnosticsSnapshot) -> OrpheusMixOutputState? {
        diagnostics.mixOutputs.first { $0.index == selectedMixerOutput } ?? diagnostics.mixOutputs.first
    }

    private func mixerInputRow(_ input: OrpheusMixInputState) -> some View {
        HStack(spacing: 10) {
            Text(input.label)
                .font(.caption.bold())
                .foregroundStyle(input.label.hasPrefix("DAW") ? .blue : .primary)
                .frame(width: 78, alignment: .leading)
            Text(input.gainLabel)
                .font(.system(.caption, design: .monospaced))
                .frame(width: 54, alignment: .trailing)
            statusPill(input.muted ? "Muted" : "Open",
                       color: input.muted ? .orange : .secondary)
                .frame(width: 64, alignment: .leading)
            Text(input.panLabel)
                .font(.system(.caption, design: .monospaced))
                .foregroundStyle(.secondary)
                .frame(width: 68, alignment: .leading)
            GeometryReader { proxy in
                RoundedRectangle(cornerRadius: 3)
                    .fill(input.muted ? Color.orange.opacity(0.25) : Color.blue.opacity(0.35))
                    .frame(width: max(3, proxy.size.width * normalizedGain(input.gain)))
            }
            .frame(height: 6)
        }
        .padding(.vertical, 3)
    }

    private func meterGroup(_ group: OrpheusMeterLevelState.Group, meters: OrpheusMeterSnapshot) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            Text(group.displayName)
                .font(.caption.bold())
                .foregroundStyle(.secondary)

            MetalMeterBankView(levels: metalLevels(for: group, meters: meters),
                               height: metalHeight(for: group))
        }
    }

    private func diagnosticMetric(_ title: String, _ value: String, raw: String) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            Text(title)
                .font(.caption)
                .foregroundStyle(.secondary)
            Text(value)
                .font(.subheadline.bold())
                .lineLimit(1)
            Text(raw)
                .font(.system(.caption2, design: .monospaced))
                .foregroundStyle(.secondary)
                .lineLimit(1)
        }
        .padding(10)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color.secondary.opacity(0.08))
        .clipShape(RoundedRectangle(cornerRadius: 8))
    }

    private func statusPill(_ text: String, color: Color) -> some View {
        Text(text)
            .font(.caption.bold())
            .lineLimit(1)
            .padding(.horizontal, 7)
            .padding(.vertical, 3)
            .background(color.opacity(0.16))
            .foregroundStyle(color)
            .clipShape(Capsule())
    }

    private func rawLine(_ title: String, _ data: Data) -> some View {
        VStack(alignment: .leading, spacing: 3) {
            Text(title)
                .font(.caption.bold())
            Text(data.orpheusHexString)
                .font(.system(.caption2, design: .monospaced))
                .foregroundStyle(.secondary)
                .textSelection(.enabled)
        }
    }

    private func hex(_ value: UInt8) -> String {
        String(format: "0x%02X", value)
    }

    private func normalizedGain(_ gain: Int16) -> CGFloat {
        if gain == Int16.min {
            return 0.02
        }
        let clamped = min(max(Double(gain), -32768), 0)
        return CGFloat((clamped + 32768) / 32768)
    }

    private func playbackProbeColor(_ path: OrpheusPlaybackPathState) -> Color {
        if path.outputActive {
            return .green
        }
        if path.outputMuted || path.probeLabel == "Mixer DAW closed" {
            return .orange
        }
        if path.dawActive {
            return .blue
        }
        return .secondary
    }

    private func metalLevels(for group: OrpheusMeterLevelState.Group,
                             meters: OrpheusMeterSnapshot) -> [MetalMeterLevel] {
        meters.levels(in: group).map { level in
            MetalMeterLevel(id: level.index,
                            label: level.label,
                            rawLabel: level.rawLabel,
                            normalized: Float(level.normalized),
                            colorFamily: metalColorFamily(for: group))
        }
    }

    private func metalHeight(for group: OrpheusMeterLevelState.Group) -> CGFloat {
        switch group {
        case .dawFeeds, .physicalOutputs:
            return 92
        case .analogInputs, .digitalInputs:
            return 78
        case .adatSend, .adatReturn:
            return 70
        }
    }

    private func metalColorFamily(for group: OrpheusMeterLevelState.Group) -> Float {
        switch group {
        case .dawFeeds:
            return 0
        case .physicalOutputs:
            return 1
        case .analogInputs, .digitalInputs:
            return 2
        case .adatSend, .adatReturn:
            return 3
        }
    }

    // MARK: - Device Settings

    private var deviceSettingsSection: some View {
        GroupBox {
            VStack(alignment: .leading, spacing: 14) {
                // Front Panel Meters — per-device (Input / Output / Follow Global)
                HStack {
                    Text("Front Meters")
                        .frame(width: 100, alignment: .leading)

                    Picker("", selection: Binding(
                        get: { viewModel.localFpMeters },
                        set: { viewModel.setLocalFpMeters($0) }
                    )) {
                        ForEach(OrpheusFpMeterLocal.allCases) { mode in
                            Text(mode.displayName).tag(mode)
                        }
                    }
                    .pickerStyle(.segmented)
                    .frame(maxWidth: 320)

                    Text("raw: \(viewModel.deviceSettings.meterMode)")
                        .font(.system(.caption2, design: .monospaced))
                        .foregroundStyle(.secondary)
                }

                // Global FP Meters — panel-wide; drives every unit set to "Follow Global"
                HStack {
                    Text("Global Meters")
                        .frame(width: 100, alignment: .leading)

                    Picker("", selection: Binding(
                        get: { viewModel.globalFpMeters },
                        set: { viewModel.setGlobalFpMeters($0) }
                    )) {
                        ForEach(OrpheusFpMeterGlobal.allCases) { mode in
                            Text(mode.displayName).tag(mode)
                        }
                    }
                    .pickerStyle(.segmented)
                    .frame(maxWidth: 220)

                    Text(viewModel.isFollowingGlobalFpMeters ? "unit follows" : "unit fixed")
                        .font(.system(.caption2, design: .monospaced))
                        .foregroundStyle(.secondary)
                }

                Divider()

                // Clock Source
                HStack {
                    Text("Clock Source")
                        .frame(width: 100, alignment: .leading)

                    Picker("", selection: Binding(
                        get: { viewModel.deviceSettings.syncSource },
                        set: { viewModel.setSyncSource($0) }
                    )) {
                        ForEach(OrpheusSyncSource.allCases) { source in
                            Text(source.displayName).tag(source.rawValue)
                        }
                    }
                    .pickerStyle(.segmented)
                    .frame(maxWidth: 300)

                    Text("raw: \(viewModel.deviceSettings.syncSource)")
                        .font(.system(.caption2, design: .monospaced))
                        .foregroundStyle(.secondary)
                }

                Divider()

                // Sample Rate
                HStack {
                    Text("Sample Rate")
                        .frame(width: 100, alignment: .leading)

                    Picker("", selection: Binding(
                        get: { viewModel.deviceSettings.sampleRate },
                        set: { viewModel.setSampleRate($0) }
                    )) {
                        ForEach(OrpheusSampleRate.allCases) { rate in
                            Text(rate.displayName).tag(rate.rawValue)
                        }
                    }
                    .pickerStyle(.menu)
                    .frame(width: 120)

                    Text("raw: \(viewModel.deviceSettings.sampleRate)")
                        .font(.system(.caption2, design: .monospaced))
                        .foregroundStyle(.secondary)

                    Spacer()

                    // Additional digital info
                    HStack(spacing: 12) {
                        HStack(spacing: 4) {
                            Text("Bit Depth:")
                                .font(.caption)
                                .foregroundStyle(.secondary)
                            Text("\(viewModel.deviceSettings.bitDepth)")
                                .font(.system(.caption, design: .monospaced))
                        }
                        HStack(spacing: 4) {
                            Text("Input Type:")
                                .font(.caption)
                                .foregroundStyle(.secondary)
                            Text("\(viewModel.deviceSettings.digitalInputType)")
                                .font(.system(.caption, design: .monospaced))
                        }
                    }
                }
            }
        } label: {
            Label("Device Settings", systemImage: "slider.horizontal.3")
                .font(.headline)
        }
    }

    // MARK: - Mic/Instrument Channels (0-3)

    private var micChannelsSection: some View {
        GroupBox {
            VStack(alignment: .leading, spacing: 12) {
                ForEach(0..<4, id: \.self) { ch in
                    micChannelRow(ch)
                    if ch < 3 { Divider() }
                }
            }
        } label: {
            Label("Mic / Instrument Inputs (AI 1-4)", systemImage: "mic.fill")
                .font(.headline)
        }
    }

    private func micChannelRow(_ ch: Int) -> some View {
        let chState = viewModel.state.channels[ch]
        let desc = OrpheusControlViewModel.channelDescriptions[ch]

        return VStack(alignment: .leading, spacing: 8) {
            HStack {
                Text("AI \(ch + 1)")
                    .font(.subheadline.bold())
                    .frame(width: 40, alignment: .leading)
                Text(desc)
                    .font(.caption)
                    .foregroundStyle(.secondary)
                Spacer()
                Text("Type: \(chState.type)")
                    .font(.system(.caption2, design: .monospaced))
                    .foregroundStyle(.secondary)
            }

            HStack(spacing: 16) {
                // Phantom (48V)
                Toggle("48V", isOn: Binding(
                    get: { viewModel.state.channels[ch].phantom },
                    set: { viewModel.setPhantom(channel: ch, enabled: $0) }
                ))
                .toggleStyle(.switch)
                .controlSize(.small)
                .frame(width: 80)

                // Overkiller
                Toggle("OVK", isOn: Binding(
                    get: { viewModel.state.channels[ch].overkiller },
                    set: { viewModel.setOverkiller(channel: ch, enabled: $0) }
                ))
                .toggleStyle(.switch)
                .controlSize(.small)
                .frame(width: 80)

                // Phase
                Toggle("Phase", isOn: Binding(
                    get: { viewModel.state.channels[ch].phase },
                    set: { viewModel.setPhase(channel: ch, enabled: $0) }
                ))
                .toggleStyle(.switch)
                .controlSize(.small)
                .frame(width: 90)

                // Mid/Side
                if ch <= 1 {
                    Toggle("M/S", isOn: Binding(
                        get: { viewModel.state.channels[ch].midSide },
                        set: { viewModel.setMidSide(channel: ch, enabled: $0) }
                    ))
                    .toggleStyle(.switch)
                    .controlSize(.small)
                    .frame(width: 80)
                }

                Spacer()

                // Filter
                Picker("Filter", selection: Binding(
                    get: { OrpheusFilterMode(rawValue: viewModel.state.channels[ch].filter) ?? .off },
                    set: { viewModel.setFilter(channel: ch, mode: $0) }
                )) {
                    ForEach(OrpheusFilterMode.allCases) { mode in
                        Text(mode.displayName).tag(mode)
                    }
                }
                .pickerStyle(.segmented)
                .frame(width: 140)
            }

            HStack(spacing: 24) {
                // Line levels
                HStack(spacing: 4) {
                    Text("In:")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                    Text(chState.lineInLevel ? "+4 dBu" : "-10 dBV")
                        .font(.system(.caption, design: .monospaced))
                }

                HStack(spacing: 4) {
                    Text("Out:")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                    Text(chState.lineOutLevel ? "+4 dBu" : "-10 dBV")
                        .font(.system(.caption, design: .monospaced))
                }

                if ch <= 3 {
                    HStack(spacing: 6) {
                        Text("Mic Gain")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                            .frame(width: 64, alignment: .leading)
                        Slider(
                            value: Binding(
                                get: { Double(viewModel.state.channels[ch].micGain) },
                                set: { viewModel.setMicGain(channel: ch, value: UInt8(clamping: Int($0.rounded()))) }
                            ),
                            in: 0...60,
                            step: 1
                        )
                        .frame(width: 160)
                        Text("\(chState.micGain)")
                            .font(.system(.caption, design: .monospaced))
                            .frame(width: 28, alignment: .trailing)
                    }
                }

                if ch <= 1 {
                    HStack(spacing: 4) {
                        Text("Impedance:")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                        Text("\(chState.impedance)")
                            .font(.system(.caption, design: .monospaced))
                    }
                }
            }
        }
        .padding(.vertical, 4)
    }

    // MARK: - Line Channels (4-7)

    private var lineChannelsSection: some View {
        GroupBox {
            VStack(alignment: .leading, spacing: 12) {
                ForEach(4..<8, id: \.self) { ch in
                    lineChannelRow(ch)
                    if ch < 7 { Divider() }
                }
            }
        } label: {
            Label("Line Inputs (AI 5-8)", systemImage: "cable.connector.horizontal")
                .font(.headline)
        }
    }

    private func lineChannelRow(_ ch: Int) -> some View {
        return VStack(alignment: .leading, spacing: 8) {
            HStack {
                Text("AI \(ch + 1)")
                    .font(.subheadline.bold())
                    .frame(width: 40, alignment: .leading)
                Text("Line")
                    .font(.caption)
                    .foregroundStyle(.secondary)
                Spacer()
            }

            HStack(spacing: 16) {
                // Overkiller
                Toggle("OVK", isOn: Binding(
                    get: { viewModel.state.channels[ch].overkiller },
                    set: { viewModel.setOverkiller(channel: ch, enabled: $0) }
                ))
                .toggleStyle(.switch)
                .controlSize(.small)
                .frame(width: 80)

                // Phase
                Toggle("Phase", isOn: Binding(
                    get: { viewModel.state.channels[ch].phase },
                    set: { viewModel.setPhase(channel: ch, enabled: $0) }
                ))
                .toggleStyle(.switch)
                .controlSize(.small)
                .frame(width: 90)

                Spacer()

                // Line In Level toggle
                Picker("In Level", selection: Binding(
                    get: { viewModel.state.channels[ch].lineInLevel },
                    set: { viewModel.setLineInLevel(channel: ch, plus4dBu: $0) }
                )) {
                    Text("+4 dBu").tag(true)
                    Text("-10 dBV").tag(false)
                }
                .pickerStyle(.segmented)
                .frame(width: 140)

                // Line Out Level toggle
                Picker("Out Level", selection: Binding(
                    get: { viewModel.state.channels[ch].lineOutLevel },
                    set: { viewModel.setLineOutLevel(channel: ch, plus4dBu: $0) }
                )) {
                    Text("+4 dBu").tag(true)
                    Text("-10 dBV").tag(false)
                }
                .pickerStyle(.segmented)
                .frame(width: 140)
            }
        }
        .padding(.vertical, 4)
    }

    // MARK: - Helpers

    private func stateCard(title: String, message: String) -> some View {
        GroupBox {
            VStack(spacing: 12) {
                Image(systemName: "exclamationmark.triangle")
                    .font(.largeTitle)
                    .foregroundStyle(.secondary)
                Text(title)
                    .font(.headline)
                Text(message)
                    .font(.callout)
                    .foregroundStyle(.secondary)
                    .multilineTextAlignment(.center)
            }
            .frame(maxWidth: .infinity)
            .padding()
        }
    }
}

#Preview {
    OrpheusControlView(connector: ASFWDriverConnector())
}
