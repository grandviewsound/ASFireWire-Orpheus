//
//  DeviceTonePlayer.swift
//  ASFW
//
//  Small HAL-output tone generator for channel-by-channel speaker testing.
//

import Foundation
import AudioToolbox
import AudioUnit
import CoreAudio

enum DeviceTonePlayerError: LocalizedError {
    case noOutputChannels
    case invalidChannelSelection
    case componentUnavailable
    case osStatus(String, OSStatus)

    var errorDescription: String? {
        switch self {
        case .noOutputChannels:
            return "Selected device has no output channels."
        case .invalidChannelSelection:
            return "Requested tone channels are outside the device output range."
        case .componentUnavailable:
            return "Unable to create the Core Audio HAL output unit."
        case let .osStatus(operation, status):
            return "\(operation) failed (\(status))."
        }
    }
}

final class DeviceTonePlayer {
    private var audioUnit: AudioUnit?
    private var sampleRate: Double = 48_000
    private var channelCount: UInt32 = 2
    private var activeChannelMask: UInt64 = 0
    private var amplitude: Float = 0.2
    private var phase: Double = 0
    private var phaseIncrement: Double = 0

    deinit {
        stop()
    }

    var isRunning: Bool {
        audioUnit != nil
    }

    func start(device: AudioWrapperDevice,
               channels: [Int],
               frequency: Double,
               amplitude: Float) throws {
        stop()

        let outputChannels = device.outputChannelCount
        guard outputChannels > 0 else {
            throw DeviceTonePlayerError.noOutputChannels
        }
        guard !channels.isEmpty,
              channels.allSatisfy({ $0 >= 0 && $0 < outputChannels }) else {
            throw DeviceTonePlayerError.invalidChannelSelection
        }

        self.sampleRate = device.sampleRate > 0 ? device.sampleRate : 48_000
        self.channelCount = UInt32(outputChannels)
        self.amplitude = amplitude
        self.phase = 0
        self.phaseIncrement = (2.0 * .pi * frequency) / self.sampleRate
        self.activeChannelMask = channels.reduce(into: UInt64(0)) { mask, channel in
            mask |= (UInt64(1) << UInt64(channel))
        }

        var desc = AudioComponentDescription(componentType: kAudioUnitType_Output,
                                             componentSubType: kAudioUnitSubType_HALOutput,
                                             componentManufacturer: kAudioUnitManufacturer_Apple,
                                             componentFlags: 0,
                                             componentFlagsMask: 0)

        guard let component = AudioComponentFindNext(nil, &desc) else {
            throw DeviceTonePlayerError.componentUnavailable
        }

        var unit: AudioUnit?
        try check(AudioComponentInstanceNew(component, &unit), operation: "AudioComponentInstanceNew")
        guard let audioUnit = unit else {
            throw DeviceTonePlayerError.componentUnavailable
        }

        var enableOutput: UInt32 = 1
        var disableInput: UInt32 = 0
        var deviceID = device.id

        try check(AudioUnitSetProperty(audioUnit,
                                       kAudioOutputUnitProperty_EnableIO,
                                       kAudioUnitScope_Output,
                                       0,
                                       &enableOutput,
                                       UInt32(MemoryLayout<UInt32>.size)),
                  operation: "Enable HAL output")
        try check(AudioUnitSetProperty(audioUnit,
                                       kAudioOutputUnitProperty_EnableIO,
                                       kAudioUnitScope_Input,
                                       1,
                                       &disableInput,
                                       UInt32(MemoryLayout<UInt32>.size)),
                  operation: "Disable HAL input")
        try check(AudioUnitSetProperty(audioUnit,
                                       kAudioOutputUnitProperty_CurrentDevice,
                                       kAudioUnitScope_Global,
                                       0,
                                       &deviceID,
                                       UInt32(MemoryLayout<AudioDeviceID>.size)),
                  operation: "Select output device")

        var callback = AURenderCallbackStruct(inputProc: deviceToneRenderCallback,
                                              inputProcRefCon: Unmanaged.passUnretained(self).toOpaque())
        try check(AudioUnitSetProperty(audioUnit,
                                       kAudioUnitProperty_SetRenderCallback,
                                       kAudioUnitScope_Input,
                                       0,
                                       &callback,
                                       UInt32(MemoryLayout<AURenderCallbackStruct>.size)),
                  operation: "Install render callback")

        var asbd = AudioStreamBasicDescription(mSampleRate: self.sampleRate,
                                               mFormatID: kAudioFormatLinearPCM,
                                               mFormatFlags: kAudioFormatFlagsNativeFloatPacked,
                                               mBytesPerPacket: UInt32(MemoryLayout<Float>.size) * self.channelCount,
                                               mFramesPerPacket: 1,
                                               mBytesPerFrame: UInt32(MemoryLayout<Float>.size) * self.channelCount,
                                               mChannelsPerFrame: self.channelCount,
                                               mBitsPerChannel: 32,
                                               mReserved: 0)
        try check(AudioUnitSetProperty(audioUnit,
                                       kAudioUnitProperty_StreamFormat,
                                       kAudioUnitScope_Input,
                                       0,
                                       &asbd,
                                       UInt32(MemoryLayout<AudioStreamBasicDescription>.size)),
                  operation: "Set client stream format")

        try check(AudioUnitInitialize(audioUnit), operation: "AudioUnitInitialize")
        try check(AudioOutputUnitStart(audioUnit), operation: "AudioOutputUnitStart")

        self.audioUnit = audioUnit
    }

    func stop() {
        guard let unit = audioUnit else {
            return
        }
        AudioOutputUnitStop(unit)
        AudioUnitUninitialize(unit)
        AudioComponentInstanceDispose(unit)
        audioUnit = nil
    }

    fileprivate func render(ioData: UnsafeMutablePointer<AudioBufferList>?,
                            frames: UInt32) -> OSStatus {
        guard let ioData else { return noErr }

        let bufferList = UnsafeMutableAudioBufferListPointer(ioData)
        let frameCount = Int(frames)
        let amp = amplitude
        let mask = activeChannelMask
        var phase = self.phase
        let increment = phaseIncrement

        if bufferList.count == 1 {
            guard let mData = bufferList[0].mData else { return noErr }
            let samplePtr = mData.assumingMemoryBound(to: Float.self)
            renderSingleBuffer(samplePtr,
                               frameCount: frameCount,
                               channelCount: Int(channelCount),
                               amp: amp,
                               mask: mask,
                               phase: &phase,
                               increment: increment)
            bufferList[0].mDataByteSize = UInt32(frameCount * Int(channelCount) * MemoryLayout<Float>.size)
        } else {
            renderNonInterleaved(bufferList,
                                 frameCount: frameCount,
                                 amp: amp,
                                 mask: mask,
                                 phase: &phase,
                                 increment: increment)
        }

        self.phase = phase
        return noErr
    }

    private func check(_ status: OSStatus, operation: String) throws {
        if status != noErr {
            throw DeviceTonePlayerError.osStatus(operation, status)
        }
    }

    private func renderSingleBuffer(_ samplePtr: UnsafeMutablePointer<Float>,
                                    frameCount: Int,
                                    channelCount: Int,
                                    amp: Float,
                                    mask: UInt64,
                                    phase: inout Double,
                                    increment: Double) {
        for frame in 0..<frameCount {
            let sample = sin(phase) * Double(amp)
            phase += increment
            if phase >= (2.0 * .pi) {
                phase -= (2.0 * .pi)
            }

            let base = frame * channelCount
            for channel in 0..<channelCount {
                samplePtr[base + channel] = ((mask & (UInt64(1) << UInt64(channel))) != 0)
                    ? Float(sample)
                    : 0
            }
        }
    }

    private func renderNonInterleaved(_ bufferList: UnsafeMutableAudioBufferListPointer,
                                      frameCount: Int,
                                      amp: Float,
                                      mask: UInt64,
                                      phase: inout Double,
                                      increment: Double) {
        for frame in 0..<frameCount {
            let sample = sin(phase) * Double(amp)
            phase += increment
            if phase >= (2.0 * .pi) {
                phase -= (2.0 * .pi)
            }

            for channel in 0..<bufferList.count {
                guard let mData = bufferList[channel].mData else { continue }
                let samplePtr = mData.assumingMemoryBound(to: Float.self)
                samplePtr[frame] = ((mask & (UInt64(1) << UInt64(channel))) != 0)
                    ? Float(sample)
                    : 0
            }
        }

        for channel in 0..<bufferList.count {
            bufferList[channel].mDataByteSize = UInt32(frameCount * MemoryLayout<Float>.size)
        }
    }
}

private let deviceToneRenderCallback: AURenderCallback = { refCon, _, _, _, frames, ioData in
    let player = Unmanaged<DeviceTonePlayer>.fromOpaque(refCon).takeUnretainedValue()
    return player.render(ioData: ioData, frames: frames)
}
