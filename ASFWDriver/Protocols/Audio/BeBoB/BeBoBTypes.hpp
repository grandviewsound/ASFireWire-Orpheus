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
// Reference: Orpheus Control Panel behavior, OrpheusModels.swift
constexpr uint8_t kAVCSubunitAudio       = 0x08;  // Audio subunit type=1, ID=0
constexpr uint8_t kAVCOpcodeVendorDep    = 0x00;  // VENDOR-DEPENDENT
constexpr uint8_t kPrismOUI0             = 0x00;
constexpr uint8_t kPrismOUI1             = 0x11;
constexpr uint8_t kPrismOUI2             = 0x98;
constexpr uint32_t kVendorDeviceCmdSize  = 15;

// Vendor opcode table — confirmed by analysis of Prism Control Panel's
// Orpheus::Device::{Get,Set,SetSyncAvc} (Apr 26 2026). Opcodes 0xA0-0xBF
// at music subunit (0x08), VENDOR-DEPENDENT (0x00), Prism OUI 00:11:98.
// CONTROL writes use ctype=0x00, STATUS reads use ctype=0x01.
constexpr uint8_t kOrpheusCmdMasterVol     = 0xA0;  // MasterVol::SetValue (s16)
constexpr uint8_t kOrpheusCmdMasterMute    = 0xA1;  // toggles +0x1b6
constexpr uint8_t kOrpheusCmdMasterEnabled = 0xA2;  // MasterVol::SetEnabled
constexpr uint8_t kOrpheusCmdMasterLock    = 0xA3;  // toggles +0x1b7
constexpr uint8_t kOrpheusCmdMeters        = 0xB0;  // Meter display mode
constexpr uint8_t kOrpheusCmdSource        = 0xB1;  // Global output source selector
constexpr uint8_t kOrpheusCmdWordclock     = 0xB2;  // Wordclock config
constexpr uint8_t kOrpheusCmdADAT          = 0xB3;  // ADAT mode (also STATUS-readable)
constexpr uint8_t kOrpheusCmdUnknownB4     = 0xB4;  // toggles +0x1b0
constexpr uint8_t kOrpheusCmdUnknownB6     = 0xB6;  // STATUS single-quadlet read (Get case 10)
constexpr uint8_t kOrpheusCmdMetersBright  = 0xB7;  // Meters::SetBrightness
constexpr uint8_t kOrpheusCmdAnalogBulk    = 0xCF;  // Bulk analog state read
constexpr uint8_t kOrpheusCmdDigitalSync   = 0xD3;  // Digital sync source (CONTROL-only)
constexpr uint8_t kOrpheusCmdDigitalBulk   = 0xDF;  // Bulk digital state read
constexpr uint8_t kOrpheusCmdMixOutDefeat  = 0xE0;  // Mix output defeated
constexpr uint8_t kOrpheusCmdMixOutSolo    = 0xE2;  // Mix output solo mask
constexpr uint8_t kOrpheusCmdMixOutMute    = 0xE3;  // Mix output mute
constexpr uint8_t kOrpheusCmdMixOutGain    = 0xE4;  // Mix output gain (s16)
constexpr uint8_t kOrpheusCmdMixInMute     = 0xE5;  // Mix input mute
constexpr uint8_t kOrpheusCmdMixInGain     = 0xE6;  // Mix input gain (s16)
constexpr uint8_t kOrpheusCmdMixInPan      = 0xE7;  // Mix input pan (s8)
constexpr uint8_t kOrpheusCmdMixInBalance  = 0xE8;  // Mix input balance (s8)
constexpr uint8_t kOrpheusCmdMixBulk       = 0xEF;  // Bulk mix output read/write (74-byte frame)
constexpr uint8_t kOrpheusCmdBulkState     = 0xBF;  // Unified bulk state STATUS read/write (Get/Set case 13)
                                                    // Returns source/wordclock/ADAT/master vol/
                                                    // mute/lock/meters/headphones in 8 bytes

// Source routing values for kOrpheusCmdSource (0xB1)
// Working old-laptop XML shows <src>1</src> — FireWire must be selected
// for DACs to play isochronous audio data.
constexpr uint8_t kOrpheusSourceFireWire = 0x01;

// ============================================================================
// AV/C Standard SignalSource (opcode 0x1A) — sync-source read/write at UNIT
// ============================================================================
// Mirrors Orpheus::Device::Sync() and SetSyncAvc(int) from the Prism Control
// Panel. Sync source is read/written as a SignalSource STATUS/CONTROL at UNIT
// addressing, with destination plug = External-plug-type 0x60, plug ID 0x07
// (or 0x08 if ADAT input enabled). 8-byte frame:
//   [01/00][FF][1A][0F][src_hi][src_lo][0x60][0x07-or-0x08]
// STATUS request uses src=0xFFFF wildcard; response[5] decodes to sync enum.
constexpr uint8_t kAVCOpcodeSignalSource    = 0x1A;
constexpr uint8_t kSignalSourceReserved0F   = 0x0F;
constexpr uint8_t kSignalSourceQueryHi      = 0xFF;  // wildcard for STATUS
constexpr uint8_t kSignalSourceQueryLo      = 0xFF;
constexpr uint8_t kSignalSourceDestPlugHi   = 0x60;  // external plug type
constexpr uint8_t kSignalSourceSyncPlugNoAdat = 0x07;
constexpr uint8_t kSignalSourceSyncPlugAdat   = 0x08;
constexpr uint32_t kSignalSourceCommandSize = 8;

// Sync-source enum (decoded from SetSyncAvc switch + Sync response decoder)
// Confirmed by analysis: Orpheus::Device::SetSyncAvc(int) builds case-by-case
// SOURCE bytes; Orpheus::Device::Sync() decodes response[5] back to these.
enum OrpheusSyncSource : std::uint8_t {
    kOrpheusSyncLocal     = 0,  // Internal master (loopback)
    kOrpheusSyncFreeRun   = 1,  // No source / free-run
    kOrpheusSyncWordclock = 2,  // External Wordclock — mutes if cable absent
    kOrpheusSyncSPDIF     = 3,  // External S/PDIF — mutes if cable absent
    kOrpheusSyncADAT      = 4,  // External ADAT  — mutes if cable absent (ADAT input must be enabled)
    kOrpheusSyncSlave     = 5,  // Slave to FW host (PC-DAW mode)
};

// ============================================================================
// Orpheus AM824 Compound Cluster Layout (for Extended Stream Format CONTROL)
// ============================================================================
// oPCR (source plug 0, device→host, recording): 5×(2ch MBLA) + 1×(1ch MIDI)
// iPCR (dest plug 0, host→device, playback):    6×(2ch MBLA) + 1×(1ch MIDI)
constexpr uint8_t kOrpheusOutputMBLAPairs = 5;  // 5 stereo pairs = 10 audio ch
constexpr uint8_t kOrpheusInputMBLAPairs  = 6;  // 6 stereo pairs = 12 audio ch
constexpr uint8_t kOrpheusAnalogChannelCount = 8;
constexpr uint8_t kOrpheusMixOutputCount  = 6;  // Line 1/2..7/8, S/PDIF, headphone pairs
constexpr uint8_t kOrpheusMixInputCount   = 12; // 12 DAW/mixer inputs per output
constexpr uint32_t kOrpheusMixBulkFrameSize = 74;
constexpr uint32_t kOrpheusMixBulkOperandLength = kOrpheusMixBulkFrameSize - 3;

} // namespace ASFW::Audio::BeBoB
