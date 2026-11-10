#pragma once

#include <limits>
#include <cstring>

#include <common/fsm_base_types.hpp>
#include "selftest_sub_state.hpp"
#include "inc/MarlinConfig.h"

struct SelftestToolOffsets_t {
    uint8_t progress;
    SelftestSubtestState_t state;
    uint8_t failed_tool_raw = 0;
    uint8_t nozzle_side_30deg = 0;
    bool failure_is_unstable = false;
    bool measured_shallow = false;
    uint8_t deviation_centi_mm = 0;

    static constexpr int16_t TOOL_OFFSET_CLEANING_TEMPERATURE = 200; //< Temperature to heat up towards for cleaning tools
    static constexpr int16_t TOOL_CALIBRATION_TEMPERATURE = 70; //< Temperature to maintain during calibration
    static constexpr int16_t FAILED_NOZZLE_CLEANING_TEMPERATURE = 280; //< Failed nozzle cleaning temperature

    constexpr SelftestToolOffsets_t(uint8_t prog = 0, SelftestSubtestState_t st = SelftestSubtestState_t::undef)
        : progress(prog)
        , state(st) {}

    constexpr SelftestToolOffsets_t(fsm::PhaseData data)
        : progress(0)
        , state(static_cast<SelftestSubtestState_t>(data[0]))
        , failed_tool_raw(data[1])
        , nozzle_side_30deg(data[2] & 0x0F)
        , failure_is_unstable(data[2] & 0x10)
        , measured_shallow(data[2] & 0x20)
        , deviation_centi_mm(data[3]) {}

    constexpr bool operator==(const SelftestToolOffsets_t &other) const {
        return progress == other.progress && state == other.state && failed_tool_raw == other.failed_tool_raw
            && nozzle_side_30deg == other.nozzle_side_30deg && failure_is_unstable == other.failure_is_unstable
            && measured_shallow == other.measured_shallow && deviation_centi_mm == other.deviation_centi_mm;
    }

    constexpr bool operator!=(const SelftestToolOffsets_t &other) const {
        return !((*this) == other);
    }

    void Pass() {
        state = SelftestSubtestState_t::ok;
        progress = 100;
    }

    void Fail() {
        state = SelftestSubtestState_t::not_good;
        progress = 100;
    }

    void Abort() {}

    constexpr fsm::PhaseData Serialize() const {
        return { { static_cast<uint8_t>(state), failed_tool_raw,
            static_cast<uint8_t>(nozzle_side_30deg | (failure_is_unstable << 4) | (measured_shallow << 5)), deviation_centi_mm } };
    }
};
