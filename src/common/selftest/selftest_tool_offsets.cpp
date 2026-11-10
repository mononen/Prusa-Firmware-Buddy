#include "selftest_tool_offsets.hpp"
#include "Marlin/src/gcode/queue.h"
#include "Marlin/src/module/stepper.h"
#include "Marlin/src/module/motion.h"
#include "marlin_server.hpp"
#include "selftest_tool_helper.hpp"
#include "Marlin/src/module/temperature.h"
#include "fanctl.hpp"
#include <module/tool_change.h>
#include <algorithm>
#include <cmath>

#include <tool_index.hpp>

using namespace selftest;
LOG_COMPONENT_REF(Selftest);

namespace {
/// @brief Set temperature to all enabled tools
void set_nozzle_temps(int16_t temp) {
    for (auto tool : PhysicalToolIndex::all()) {
        if (is_tool_selftest_enabled(tool, AllTools {})) { // set temperature on all tools, its not possible to calibrate just one tool
            thermalManager.setTargetHotend(temp, tool);
        }
    }
}

/// @brief Check temperature of all enabled tools is at target
bool all_nozzles_at_target() {
    for (auto tool : PhysicalToolIndex::all()) {
        if (is_tool_selftest_enabled(tool, AllTools {})) { // check temperature on all tools, its not possible to calibrate just one tool
            if (!Hotend::for_tool(tool).is_nozzle_temp_reached()) {
                return false;
            }
        }
    }
    return true;
}
}; // namespace

CSelftestPart_ToolOffsets::CSelftestPart_ToolOffsets(IPartHandler &state_machine, const ToolOffsetsConfig_t &config, SelftestToolOffsets_t &result)
    : state_machine(state_machine)
    , result(result)
    , config(config) {}

CSelftestPart_ToolOffsets::~CSelftestPart_ToolOffsets() {
    if (failed_nozzle) {
        thermalManager.setTargetHotend(0, failed_nozzle->tool);
    }
}

LoopResult CSelftestPart_ToolOffsets::state_ask_user_confirm_start() {
    IPartHandler::SetFsmPhase(PhasesSelftest::ToolOffsets_wait_user_confirm_start);
    return LoopResult::RunNext;
}

LoopResult CSelftestPart_ToolOffsets::state_clean_nozzle_start() {
    IPartHandler::SetFsmPhase(PhasesSelftest::ToolOffsets_wait_move_away);
    disable_all_steppers(); // Let the user operate tools, pull out the filament if required
    set_nozzle_temps(0);

    return LoopResult::RunNext;
}

LoopResult CSelftestPart_ToolOffsets::state_move_away() {
    IPartHandler::SetFsmPhase(PhasesSelftest::ToolOffsets_wait_user_clean_nozzle_cold);
    // we'll ask user to clean nozzle and put on sheet - so give him some space
    do_z_clearance(100);
    return LoopResult::RunNext;
}

LoopResult CSelftestPart_ToolOffsets::state_clean_nozzle() {
    const auto button_pressed = state_machine.GetButtonPressed();

    if (button_pressed == Response::Continue) {
        set_nozzle_temps(SelftestToolOffsets_t::TOOL_CALIBRATION_TEMPERATURE);
        quick_cooling.set_active(true);
        return LoopResult::RunNext;
    }

    if (IPartHandler::GetFsmPhase() == PhasesSelftest::ToolOffsets_wait_user_clean_nozzle_hot) {
        // nozzle is hot or heating up
        if (button_pressed == Response::Cooldown) {
            IPartHandler::SetFsmPhase(PhasesSelftest::ToolOffsets_wait_user_clean_nozzle_cold);
            set_nozzle_temps(SelftestToolOffsets_t::TOOL_CALIBRATION_TEMPERATURE);
            quick_cooling.set_active(true);
        }
    } else if (IPartHandler::GetFsmPhase() == PhasesSelftest::ToolOffsets_wait_user_clean_nozzle_cold) {
        // nozzle is cold or cooling down
        if (button_pressed == Response::Heatup) {
            IPartHandler::SetFsmPhase(PhasesSelftest::ToolOffsets_wait_user_clean_nozzle_hot);
            set_nozzle_temps(SelftestToolOffsets_t::TOOL_OFFSET_CLEANING_TEMPERATURE);
            quick_cooling.set_active(false);
        }
    }

    return LoopResult::RunCurrent;
}

LoopResult CSelftestPart_ToolOffsets::state_ask_user_install_sheet() {
    IPartHandler::SetFsmPhase(PhasesSelftest::ToolOffsets_wait_user_install_sheet);
    return LoopResult::RunNext;
}

LoopResult CSelftestPart_ToolOffsets::state_wait_user() {
    if (state_machine.GetButtonPressed() != Response::Continue) {
        return LoopResult::RunCurrent;
    }
    return LoopResult::RunNext;
}

LoopResult CSelftestPart_ToolOffsets::state_home_park() {
    IPartHandler::SetFsmPhase(PhasesSelftest::ToolOffsets_pin_install_prepare);

    // We need to be Z homed, so that we can find the pin
    marlin_server::enqueue_gcode("G28 Z O");

    // Park the nozzle out of the way, the user is about to remove the steel sheet.
    marlin_server::enqueue_gcode("P0 L0 D0");

    // Ensure tool will not hit calibration pin once installed
    marlin_server::enqueue_gcode("G1 Z30");

    return LoopResult::RunNext;
}

LoopResult CSelftestPart_ToolOffsets::state_wait_moves_done() {
    if (queue.has_commands_queued() || planner.processing()) {
        return LoopResult::RunCurrent;
    }
    return LoopResult::RunNext;
}

LoopResult CSelftestPart_ToolOffsets::state_ask_user_install_pin() {
    IPartHandler::SetFsmPhase(PhasesSelftest::ToolOffsets_wait_user_install_pin);
    return LoopResult::RunNext;
}

LoopResult CSelftestPart_ToolOffsets::state_wait_stable_temp() {
    IPartHandler::SetFsmPhase(PhasesSelftest::ToolOffsets_wait_stable_temp);
    if (all_nozzles_at_target()) {
        quick_cooling.set_active(false);
        return LoopResult::RunNext;
    }
    return LoopResult::RunCurrent;
}

LoopResult CSelftestPart_ToolOffsets::state_calibrate() {
    IPartHandler::SetFsmPhase(PhasesSelftest::ToolOffsets_wait_calibrate);
    return LoopResult::RunNext;
}

// The FSM must publish the calibration phase before the blocking measurement.
LoopResult CSelftestPart_ToolOffsets::state_finish_calibration() {
    if (axis_unhomed_error()) {
        return LoopResult::Fail;
    }
    const auto measured = pin_calibration.measure_remaining_tools();
    if (measured) {
        return LoopResult::RunNext;
    }
    failed_nozzle = measured.error();
    result.failed_tool_raw = failed_nozzle->tool.to_raw();
    result.nozzle_side_30deg = failed_nozzle->nozzle_side_30deg;
    result.failure_is_unstable = failed_nozzle->reason == pin_calibration::NozzleFailureReason::xy_probe_unstable;
    result.measured_shallow = failed_nozzle->bore_depth_mm == pin_calibration::SHALLOW_BORE_DEPTH_MM;
    result.deviation_centi_mm = static_cast<uint8_t>(std::clamp<long>(std::lround(failed_nozzle->deviation_mm * 100), 0, 255));
    tool_change(NoTool {}, tool_return_t::no_return);
    mapi::park(mapi::get_parking_position(mapi::ParkPosition::filament_change));
    IPartHandler::SetFsmPhase(PhasesSelftest::ToolOffsets_wait_user_clean_failed_nozzle_cold);
    return LoopResult::RunNext;
}

LoopResult CSelftestPart_ToolOffsets::retry_failed_nozzle() {
    thermalManager.setTargetHotend(SelftestToolOffsets_t::TOOL_CALIBRATION_TEMPERATURE, failed_nozzle->tool);
    quick_cooling.set_active(true);
    failed_nozzle.reset();
    result.failed_tool_raw = 0;
    result.nozzle_side_30deg = 0;
    result.failure_is_unstable = false;
    result.measured_shallow = false;
    result.deviation_centi_mm = 0;
    return LoopResult::GoToMark0;
}

LoopResult CSelftestPart_ToolOffsets::state_clean_failed_nozzle() {
    if (!failed_nozzle) {
        return LoopResult::RunNext;
    }
    switch (state_machine.GetButtonPressed()) {
    case Response::Retry:
        return retry_failed_nozzle();
    case Response::RetryShallower:
        pin_calibration.use_shallow_bore_depth(failed_nozzle->tool);
        return retry_failed_nozzle();
    case Response::Heatup:
        IPartHandler::SetFsmPhase(PhasesSelftest::ToolOffsets_wait_user_clean_failed_nozzle_hot);
        thermalManager.setTargetHotend(SelftestToolOffsets_t::FAILED_NOZZLE_CLEANING_TEMPERATURE, failed_nozzle->tool);
        quick_cooling.set_active(false);
        break;
    case Response::Cooldown:
        IPartHandler::SetFsmPhase(PhasesSelftest::ToolOffsets_wait_user_clean_failed_nozzle_cold);
        thermalManager.setTargetHotend(SelftestToolOffsets_t::TOOL_CALIBRATION_TEMPERATURE, failed_nozzle->tool);
        quick_cooling.set_active(true);
        break;
    default:
        break;
    }
    return LoopResult::RunCurrent;
}

LoopResult CSelftestPart_ToolOffsets::state_final_park() {
    IPartHandler::SetFsmPhase(PhasesSelftest::ToolOffsets_wait_move_away);
    // Let user uninstall the pin and put the steel sheet back
    marlin_server::enqueue_gcode("P0"); // Park tool
    marlin_server::enqueue_gcode("G27 Z40 P1"); // Park head, high enough to get the sheet in
    marlin_server::enqueue_gcode("M18"); // Disable steppers
    return LoopResult::RunNext;
}

LoopResult CSelftestPart_ToolOffsets::state_ask_user_remove_pin() {
    IPartHandler::SetFsmPhase(PhasesSelftest::ToolOffsets_wait_user_remove_pin);
    return LoopResult::RunNext;
}
