#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

extern "C" {
#include "app_telem.h"
#include "app_main.h"
#include "robot_protocol.h"
#include "hip_control.h"
}

extern "C" {
extern uint32_t g_hal_tick;
extern bool g_adc_ok;
extern float g_adc_voltage;
extern uint8_t g_app_link_last_type;
extern uint16_t g_app_link_last_len;
extern uint8_t g_app_link_last_payload[ROBOT_FRAME_MAX_PAYLOAD];
void hip_control_fake_set_states(const hip_state_t *left, const hip_state_t *right);
void hip_control_fake_set_commands(const hip_command_t *left, const hip_command_t *right);
void hip_control_fake_set_faults(uint32_t faults);
void motion_control_fake_set_hip_phase_progress(uint8_t pct);
}

TEST_CASE("app_telem sends v3 telemetry with hip fields", "[telem][hip]") {
    g_hal_tick = 0U;
    app_telem_init();

    hip_state_t left = {};
    hip_state_t right = {};
    left.valid = 1U;
    right.valid = 1U;
    left.pos_rev = 0.12f;
    right.pos_rev = 0.34f;
    left.vel_rev_s = 0.5f;
    right.vel_rev_s = -0.25f;
    left.height_m = 0.6f;
    right.height_m = 0.62f;
    left.height_dot_m_s = 0.01f;
    right.height_dot_m_s = 0.02f;
    left.torque_nm = 1.2f;
    right.torque_nm = 1.4f;
    left.last_rx_ms = 100U;
    right.last_rx_ms = 120U;
    hip_control_fake_set_states(&left, &right);

    hip_command_t cmd_left = {0.2f, 0.1f, 1.0f};
    hip_command_t cmd_right = {0.3f, -0.1f, 1.5f};
    hip_control_fake_set_commands(&cmd_left, &cmd_right);
    hip_control_fake_set_faults(HIP_FAULT_BUS_OFF);
    motion_control_fake_set_hip_phase_progress(42U);

    g_adc_ok = true;
    g_adc_voltage = 12.3f;

    g_hal_tick = 500U;
    app_telem_tick(500U);

    REQUIRE(g_app_link_last_type == ROBOT_MSG_TELEM_FRAME_V3);
    REQUIRE(g_app_link_last_len == sizeof(robot_telem_v3_t));

    const auto *telem = reinterpret_cast<const robot_telem_v3_t *>(g_app_link_last_payload);
    CHECK(telem->v2.version == 3U);
    CHECK(telem->hip_left_pos_rev == Catch::Approx(0.12f));
    CHECK(telem->hip_right_pos_rev == Catch::Approx(0.34f));
    CHECK(telem->hip_left_cmd_pos_rev == Catch::Approx(0.2f));
    CHECK(telem->hip_right_cmd_pos_rev == Catch::Approx(0.3f));
    CHECK(telem->hip_phase_progress_pct == 42U);
    CHECK((telem->v2.faults & ROBOT_FAULT_HIP_BUS_OFF) != 0U);
}
