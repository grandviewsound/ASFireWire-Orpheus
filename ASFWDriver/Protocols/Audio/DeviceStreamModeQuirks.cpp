// SPDX-License-Identifier: LGPL-3.0-or-later
// Copyright (c) 2024 ASFireWire Project
//
// DeviceStreamModeQuirks.cpp - Vendor/model stream mode overrides

#include "DeviceStreamModeQuirks.hpp"

namespace ASFW::Audio::Quirks {

namespace {
constexpr uint32_t kApogeeVendorId = 0x0003DB;
constexpr uint32_t kApogeeDuetModelId = 0x01DDDD;

// Focusrite DICE devices — Linux kernel dice-stream.c unconditionally uses CIP_BLOCKING.
constexpr uint32_t kFocusriteVendorId = 0x00130e;
constexpr uint32_t kSPro24DspModelId  = 0x000008;

// Prism Sound Orpheus (BeBoB/BridgeCo DM1500) — Linux firewire-bebob always uses CIP_BLOCKING.
constexpr uint32_t kPrismSoundVendorId = 0x001198;
constexpr uint32_t kOrpheusModelId     = 0x010048;

// Orpheus post-reset stabilization delay before AV/C discovery.
//
// History: was 5000 ms — a band-aid added when early AV/C commands appeared to
// fail. That symptom was almost certainly the AR-WAKE drop bug silently dropping
// the device's FCP responses, NOT the device being un-ready: Apple attaches the
// same Orpheus in ~1.3 s with NO blanket wait, and the device answers in 1–2 ms
// once responses aren't dropped. Reduced to 1000 ms (small settle margin vs
// Apple's zero). If a cold attach regresses (early discovery FCP timeouts), raise
// back toward 2000–5000 ms — the test data reveals the real settle requirement.
constexpr uint32_t kOrpheusInitDelayMs = 1000;
} // namespace

std::optional<Model::StreamMode> LookupForcedStreamMode(
    uint32_t vendorId,
    uint32_t modelId) noexcept {
    // Apogee Duet quirk:
    // - Discovery reports/supports non-blocking, and host playback can work in that mode.
    // - Observed device output stream cadence is blocking.
    // Force blocking so host/device cadence stays aligned and stream sync remains stable.
    if (vendorId == kApogeeVendorId && modelId == kApogeeDuetModelId) {
        return Model::StreamMode::kBlocking;
    }

    // Focusrite Saffire Pro 24 DSP (DICE):
    // Linux kernel DICE driver unconditionally uses CIP_BLOCKING (dice-stream.c:508).
    // DICE devices expect blocking cadence (8 samples/packet + NO-DATA packets).
    if (vendorId == kFocusriteVendorId && modelId == kSPro24DspModelId) {
        return Model::StreamMode::kBlocking;
    }

    // Prism Sound Orpheus (BeBoB/BridgeCo DM1500):
    // Linux firewire-bebob driver always sets CIP_BLOCKING (bebob_stream.c).
    // BridgeCo devices expect blocking cadence (8 data blocks/packet).
    if (vendorId == kPrismSoundVendorId && modelId == kOrpheusModelId) {
        return Model::StreamMode::kBlocking;
    }

    return std::nullopt;
}

std::optional<uint32_t> LookupInitDelayMs(
    uint32_t vendorId,
    uint32_t modelId) noexcept {
    if (vendorId == kPrismSoundVendorId && modelId == kOrpheusModelId) {
        return kOrpheusInitDelayMs;
    }

    return std::nullopt;
}

} // namespace ASFW::Audio::Quirks
