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
// AV/C Extended Stream Format Information (opcode 0x2F)
// ============================================================================

constexpr uint8_t kAVCOpcodeExtStreamFmt = 0x2F;

// Subfunction 0xC0: get/set the currently active stream format.
// Used for both CONTROL (set) and STATUS (get current).
// Reference: AV/C Extended Stream Format spec TA 2001007,
//            Linux sound/firewire/lib.c avc_stream_set_format()
constexpr uint8_t kExtStreamFmtSubfuncCurrent = 0xC0;

// Unit PCR plug address type
constexpr uint8_t kPlugAddrUnit = 0x00;

// Plug directions (AV/C convention, from the device's perspective)
//   OUTPUT: device transmits on oPCR → host receives (recording)
//   INPUT:  device receives on iPCR  ← host transmits (playback)
constexpr uint8_t kPlugDirOutput = 0x00;
constexpr uint8_t kPlugDirInput  = 0x01;

// ============================================================================
// IEC 61883-6 Compound AM824 Format Compound
// ============================================================================

// Hierarchy root marker for IEC 61883-6 compound AM824
constexpr uint8_t kAM824CompoundMarker = 0x90;

// Sample Frequency Codes (IEC 61883-6 Table 4)
// These map directly to the 3-bit SFC field in the Compound AM824 format compound
// (opcode 0x2F Extended Stream Format Info) and in CIP FDF bytes.
// Reference: IEC 61883-6 §8.2.2, Linux amdtp-stream.h CIP_SFC_*
constexpr uint8_t kSFC_32000 = 0x00;  // CIP_SFC_32000
constexpr uint8_t kSFC_44100 = 0x01;  // CIP_SFC_44100
constexpr uint8_t kSFC_48000 = 0x02;  // CIP_SFC_48000
constexpr uint8_t kSFC_88200 = 0x03;  // CIP_SFC_88200
constexpr uint8_t kSFC_96000 = 0x04;  // CIP_SFC_96000
constexpr uint8_t kSFC_176400 = 0x05; // CIP_SFC_176400
constexpr uint8_t kSFC_192000 = 0x06; // CIP_SFC_192000

// AM824 format info labels (compound format entries, NOT wire labels)
// These are the label values used in AV/C Extended Stream Format compound format info entries.
// Wire labels are different: MBLA=0x40, MIDI=0x80 (see AM824Encoder.hpp).
constexpr uint8_t kAM824LabelMBLA = 0x06;  // Multi-Bit Linear Audio (PCM)
constexpr uint8_t kAM824LabelMIDI = 0x0D;  // MIDI conformant

// ============================================================================
// Prism Sound Orpheus Channel Counts at 48kHz
// ============================================================================
// oPCR[0]: 8 analog inputs + 2 S/PDIF inputs = 10 audio channels + 1 MIDI = DBS=11
// iPCR[0]: 8 analog outputs + 2 S/PDIF outputs = 10 audio channels + 1 MIDI = DBS=11
//          (Headphone pair is on iPCR[1] via Music subunit dest plug 1)
//
// The Orpheus firmware IGNORES SetFormat (returns NOT_IMPLEMENTED for all
// Extended Stream Format CONTROL commands — confirmed by fw_diag Phase 3c).
//
// Music subunit dest plug 0 (fw_diag Phase 3b) definitively reports the iPCR[0]
// format: 5 × (2ch MBLA) + 1 × (1ch MIDI) = 10 audio + 1 MIDI = DBS=11.
// This matches the oPCR direction (also DBS=11, confirmed by packet sniffer).

// oPCR (device output, recording direction)
constexpr uint8_t kOrpheusOutputAudioChannels = 10;  // 8 analog + 2 S/PDIF
constexpr uint8_t kOrpheusOutputMidiChannels  = 1;
constexpr uint8_t kOrpheusOutputChannels      = 11;  // DBS = 10 audio + 1 MIDI

// iPCR (device input, playback direction)
constexpr uint8_t kOrpheusInputAudioChannels = 10;   // 8 analog + 2 S/PDIF (4 pairs + 1 pair)
constexpr uint8_t kOrpheusInputMidiChannels  = 1;
constexpr uint8_t kOrpheusInputChannels      = 11;   // DBS = 10 audio + 1 MIDI

// ============================================================================
// FCP Transport
// ============================================================================

// IEEE 1394 FCP command register address (CSR core, fixed per spec)
constexpr uint64_t kFCPCommandAddress = 0xFFFFF0000B00ULL;

// ============================================================================
// Command Layout
// ============================================================================

// AV/C frame header: [ctype][subunit][opcode][subfunction][addr_type][dir][reserved][plug_num]
constexpr uint32_t kExtStreamFmtHeaderSize = 8;

// Compound AM824 format: [marker][sfc][reserved][entry_count] + N×[label][channels]
// 1-entry: 4 header + 2 = 6 bytes
// 2-entry: 4 header + 2 + 2 = 8 bytes (MBLA + MIDI)
constexpr uint32_t kAM824FormatCompound1EntrySize = 6;
constexpr uint32_t kAM824FormatCompound2EntrySize = 8;

// Total command sizes
constexpr uint32_t kSetFormat1EntryCommandSize = kExtStreamFmtHeaderSize + kAM824FormatCompound1EntrySize;  // 14
constexpr uint32_t kSetFormat2EntryCommandSize = kExtStreamFmtHeaderSize + kAM824FormatCompound2EntrySize;  // 16

} // namespace ASFW::Audio::BeBoB
