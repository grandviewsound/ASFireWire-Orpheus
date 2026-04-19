// SPDX-License-Identifier: LGPL-3.0-or-later
// BeBoB (BridgeCo BeBoB) protocol constants for Prism Sound Orpheus.
// Reference: Linux sound/firewire/bebob/, AV/C Extended Stream Format spec TA 2001007

#pragma once

#include <cstdint>

namespace ASFW::Audio::BeBoB {

// ============================================================================
// Device Identification
// ============================================================================

constexpr uint32_t kPrismSoundVendorId = 0x00001198;
constexpr uint32_t kOrpheusModelId     = 0x00010048;

// ============================================================================
// AV/C Unit Plug Signal Format (opcodes 0x18 / 0x19)
// ============================================================================
// Unit-level commands that query/set plug formats.
// The Orpheus supports these at unit level (STATUS returns STABLE 0x90 0x02).
//
// However, Apple's driver uses Extended Stream Format (opcode 0x2F) at the
// Music Subunit level (0x60), NOT unit-level plug signal format. The Orpheus
// does NOT support 0x2F at unit level (0xFF), but DOES support it at music
// subunit level — confirmed by dtrace of Apple's working driver (Apr 2026).
//
// We keep 0x18/0x19 for the post-CMP SetSampleRate step (matches Apple's
// sequence) but primary format setup uses 0x2F at music subunit.
//
// Reference: AV/C General Specification, IEC 61883-1

constexpr uint8_t kAVCOpcodeOutputPlugSignalFmt = 0x18;  // OUTPUT PLUG SIGNAL FORMAT
constexpr uint8_t kAVCOpcodeInputPlugSignalFmt  = 0x19;  // INPUT PLUG SIGNAL FORMAT

// Plug directions (AV/C convention, from the device's perspective)
//   OUTPUT: device transmits on oPCR → host receives (recording)
//   INPUT:  device receives on iPCR  ← host transmits (playback)
constexpr uint8_t kPlugDirOutput = 0x00;
constexpr uint8_t kPlugDirInput  = 0x01;

// ============================================================================
// AV/C Plug Signal Format Fields
// ============================================================================

// Format byte: AM824 compound (bits [7:6] = 10, bits [5:0] = 0x10)
constexpr uint8_t kPlugSignalFmtAM824 = 0x90;

// Sample Frequency Codes (IEC 61883-6 Table 4 / AV/C Plug Signal Format)
// Used in both CIP FDF bytes and plug signal format frequency field.
// Reference: IEC 61883-6 §8.2.2
constexpr uint8_t kSFC_32000 = 0x00;
constexpr uint8_t kSFC_44100 = 0x01;
constexpr uint8_t kSFC_48000 = 0x02;
constexpr uint8_t kSFC_88200 = 0x03;
constexpr uint8_t kSFC_96000 = 0x04;
constexpr uint8_t kSFC_176400 = 0x05;
constexpr uint8_t kSFC_192000 = 0x06;

// ============================================================================
// Prism Sound Orpheus Channel Counts at 48kHz
// ============================================================================
// oPCR[0]: 8 analog inputs + 2 S/PDIF inputs = 10 audio channels + 1 MIDI = DBS=11
// iPCR[0]: 8 analog outputs + 2 S/PDIF outputs + 2 headphone = 12 audio + 1 MIDI = DBS=13
//
// Extended Stream Format CONTROL at music subunit level (0x60) is how Apple
// configures the Orpheus (dtrace Apr 2026). Unit-level 0x2F is NOT_IMPLEMENTED.
//
// AVC discovery Input Plug 0 reports 13 channels:
//   6 × (2ch MBLA) + 1 × (1ch MIDI) = 12 audio + 1 MIDI = DBS=13
// The 6th audio pair is the headphone output (Phone), included in the same iPCR[0] stream.
// oPCR direction: 5 × (2ch MBLA) + 1 × (1ch MIDI) = 10 audio + 1 MIDI = DBS=11 (asymmetric).
// Ground truth: macOS 11 ioreg shows 12 output / 10 input channels.

// oPCR (device output, recording direction)
constexpr uint8_t kOrpheusOutputAudioChannels = 10;  // 8 analog + 2 S/PDIF
constexpr uint8_t kOrpheusOutputMidiChannels  = 1;
constexpr uint8_t kOrpheusOutputChannels      = 11;  // DBS = 10 audio + 1 MIDI

// iPCR (device input, playback direction)
constexpr uint8_t kOrpheusInputAudioChannels = 12;   // 8 analog + 2 S/PDIF + 2 headphone
constexpr uint8_t kOrpheusInputMidiChannels  = 1;
constexpr uint8_t kOrpheusInputChannels      = 13;   // DBS = 12 audio + 1 MIDI

// ============================================================================
// FCP Transport
// ============================================================================

// IEEE 1394 FCP command register address (CSR core, fixed per spec)
constexpr uint64_t kFCPCommandAddress = 0xFFFFF0000B00ULL;

// ============================================================================
// Plug Signal Format Command Layout
// ============================================================================
// AV/C Unit Plug Signal Format (opcode 0x18/0x19):
//   [ctype][subunit=0xFF][opcode][plug_id][format][frequency][0xFF][0xFF]
// Total: 8 bytes
constexpr uint32_t kPlugSignalFmtCommandSize = 8;

// ============================================================================
// Vendor-Dependent Command Layout (Prism Sound proprietary)
// ============================================================================
// AV/C Vendor-Dependent (opcode 0x00) via Audio Subunit:
//   [ctype][subunit=0x08][opcode=0x00][OUI_0][OUI_1][OUI_2][cmd][value][0xFF x7]
// Total: 15 bytes
//
// Reference: IDA Pro decompilation of Orpheus Control Panel, OrpheusModels.swift
constexpr uint8_t kAVCSubunitAudio       = 0x08;  // Audio subunit type=1, ID=0
constexpr uint8_t kAVCOpcodeVendorDep    = 0x00;  // VENDOR-DEPENDENT
constexpr uint8_t kPrismOUI0             = 0x00;
constexpr uint8_t kPrismOUI1             = 0x11;
constexpr uint8_t kPrismOUI2             = 0x98;
constexpr uint32_t kVendorDeviceCmdSize  = 15;

// Device command bytes (no channel parameter)
constexpr uint8_t kOrpheusCmdSource      = 0xB1;  // Global output source selector
constexpr uint8_t kOrpheusCmdMeters      = 0xB0;  // Meter display mode

// Source routing values for kOrpheusCmdSource (0xB1)
// Working old-laptop XML shows <src>1</src> — FireWire must be selected
// for DACs to play isochronous audio data.
constexpr uint8_t kOrpheusSourceFireWire = 0x01;

// ============================================================================
// Orpheus AM824 Compound Cluster Layout (for Extended Stream Format CONTROL)
// ============================================================================
// oPCR (source plug 0, device→host, recording): 5×(2ch MBLA) + 1×(1ch MIDI)
// iPCR (dest plug 0, host→device, playback):    6×(2ch MBLA) + 1×(1ch MIDI)
constexpr uint8_t kOrpheusOutputMBLAPairs = 5;  // 5 stereo pairs = 10 audio ch
constexpr uint8_t kOrpheusInputMBLAPairs  = 6;  // 6 stereo pairs = 12 audio ch

} // namespace ASFW::Audio::BeBoB
