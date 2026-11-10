/**
 * @file G425.hpp
 * @brief Some code that is used by G425 and selftest_tool_offsets
 */

#pragma once

#include <bitset>
#include <expected>
#include <optional>
#include <Marlin/src/core/types.h>
#include <tool_index.hpp>
#include <utils/storage/strong_index_array.hpp>

namespace pin_calibration {

/// Depth below the measured pin top at which the XY touches run [mm]
inline constexpr float DEFAULT_BORE_DEPTH_MM = 1.0f;
/// Depth for nozzles that are not round at the default depth [mm]
inline constexpr float SHALLOW_BORE_DEPTH_MM = 0.5f;

enum class NozzleFailureReason : uint8_t {
    not_round,
    xy_probe_unstable,
};

struct NozzleFailure {
    PhysicalToolIndex tool;
    NozzleFailureReason reason;
    /// Nozzle side in 30-degree steps: right = 0, back = 3, left = 6, front = 9.
    uint8_t nozzle_side_30deg;
    /// Distance outside the average radius [mm], or zero for unstable probing.
    float deviation_mm;
    /// Depth below the pin top [mm].
    float bore_depth_mm;
};

/// Measures tool offsets against the pin. Earlier results remain available for retries.
/// Restores saved offsets if calibration does not complete.
class PinCalibration {
public:
    PinCalibration() = default;
    PinCalibration(const PinCalibration &) = delete;
    PinCalibration &operator=(const PinCalibration &) = delete;
    ~PinCalibration();

    /// Use shallow XY touches for this tool in all later measurements.
    void use_shallow_bore_depth(PhysicalToolIndex tool);

    /// Measure remaining enabled tools and save offsets. Requires homed axes and the pin.
    /// On failure, the tool stays picked at safe height and later tools are not measured.
    std::expected<void, NozzleFailure> measure_remaining_tools();

private:
    float bore_depth_mm(PhysicalToolIndex tool) const;

    StrongIndexArray<std::optional<MachinePosXYZ>, PhysicalToolIndex::count, PhysicalToolIndex, PhysicalToolIndex::to_raw_static> centers;
    std::bitset<PhysicalToolIndex::count> shallow_tools;
    bool offsets_zeroed = false;
    bool completed = false;
};

} // namespace pin_calibration

