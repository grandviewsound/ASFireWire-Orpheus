//
//  AudioDebugViewModel.swift
//  ASFW
//
//  Created for ASFireWire Audio Debugging.
//

import Foundation
import Combine
import CoreAudio

class AudioDebugViewModel: ObservableObject {
    @Published var devices: [AudioWrapperDevice] = []
    @Published var selectedDevice: AudioWrapperDevice?
    @Published var selectedDeviceStreams: [AudioStream] = []
    @Published var toneFrequency: Double = 1000
    @Published var toneAmplitude: Double = 0.20
    @Published var activeToneLabel: String?
    @Published var toneError: String?
    
    // Auto-select ASFW device if found
    private let targetDeviceName = "FireWire" // Adjust based on your driver's actual name
    private let tonePlayer = DeviceTonePlayer()
    
    init() {
        refreshDevices()
    }

    deinit {
        tonePlayer.stop()
    }
    
    func refreshDevices() {
        devices = AudioSystem.shared.devices
        
        // Try to find our driver device
        if let target = devices.first(where: { $0.transportType == .fireWire || $0.name.contains("ASFW") || $0.name.contains("FireWire") }) {
            selectDevice(target)
        } else if let first = devices.first {
            selectDevice(first)
        }
    }
    
    func selectDevice(_ device: AudioWrapperDevice) {
        stopTone()
        selectedDevice = device
        selectedDeviceStreams = device.inputStreams + device.outputStreams
    }

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
        guard let device = selectedDevice else {
            toneError = "Select a device first."
            return
        }

        do {
            try tonePlayer.start(device: device,
                                 channels: channels,
                                 frequency: toneFrequency,
                                 amplitude: Float(toneAmplitude))
            activeToneLabel = label
            toneError = nil
        } catch {
            activeToneLabel = nil
            toneError = error.localizedDescription
        }
    }

    func stopTone() {
        tonePlayer.stop()
        activeToneLabel = nil
    }
}
