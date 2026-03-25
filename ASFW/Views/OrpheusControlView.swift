import SwiftUI

struct OrpheusControlView: View {
    @StateObject private var viewModel: OrpheusControlViewModel

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
                    deviceSettingsSection
                    micChannelsSection
                    lineChannelsSection
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

                Button {
                    viewModel.refresh()
                } label: {
                    Label("Refresh", systemImage: "arrow.clockwise")
                }
                .disabled(viewModel.isLoading)
            }
        } label: {
            Label("Status", systemImage: "hifispeaker.2.fill")
                .font(.headline)
        }
    }

    // MARK: - Device Settings

    private var deviceSettingsSection: some View {
        GroupBox {
            VStack(alignment: .leading, spacing: 14) {
                // Front Panel Meters
                HStack {
                    Text("Front Meters")
                        .frame(width: 100, alignment: .leading)

                    Picker("", selection: Binding(
                        get: { viewModel.deviceSettings.meterMode },
                        set: { viewModel.setMeterMode($0) }
                    )) {
                        ForEach(OrpheusMeterMode.allCases) { mode in
                            Text(mode.displayName).tag(mode.rawValue)
                        }
                    }
                    .pickerStyle(.segmented)
                    .frame(maxWidth: 300)

                    Text("raw: \(viewModel.deviceSettings.meterMode)")
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
                    HStack(spacing: 4) {
                        Text("Mic Gain:")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                        Text("\(chState.micGain)")
                            .font(.system(.caption, design: .monospaced))
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
        let chState = viewModel.state.channels[ch]

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
