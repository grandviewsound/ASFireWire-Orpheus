// SPDX-License-Identifier: LGPL-3.0-or-later
// Copyright (c) 2024 ASFireWire Project
//
// DeviceStreamModeQuirks.hpp - Vendor/model stream mode overrides

#pragma once

#include "../../Audio/Model/ASFWAudioDevice.hpp"
#include <cstdint>
#include <optional>

namespace ASFW::Audio::Quirks {

/// Return a forced stream mode for known misreporting devices.
/// Empty means "no override".
[[nodiscard]] std::optional<Model::StreamMode> LookupForcedStreamMode(
    uint32_t vendorId,
    uint32_t modelId) noexcept;

/// U5 — Return a device-specific post-reset stabilization delay (ms) to apply
/// before AV/C discovery probing. Some devices (e.g. Orpheus) cannot answer
/// AV/C commands during their hardware-init window. Empty means "no special
/// delay" (callers use their own default, e.g. 0 on first attach / 500 on
/// re-probe). Keeps the per-model timing out of the discovery flow.
[[nodiscard]] std::optional<uint32_t> LookupInitDelayMs(
    uint32_t vendorId,
    uint32_t modelId) noexcept;

[[nodiscard]] constexpr const char* StreamModeToString(Model::StreamMode mode) noexcept {
    return (mode == Model::StreamMode::kBlocking) ? "blocking" : "non-blocking";
}

} // namespace ASFW::Audio::Quirks

