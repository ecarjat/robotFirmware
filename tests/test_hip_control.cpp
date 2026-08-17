#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <cstring>

#include "app_config.h"
#include "hip_control.h"
#include "hip_kinematics.h"
#include "main.h"
#include "stm32h7xx_hal_fdcan.h"

namespace {

constexpr float kPi = 3.14159265358979323846f;

constexpr uint8_t kNodeLeft = 0x03;
constexpr uint8_t kNodeRight = 0x04;
constexpr uint8_t kCmdSetAxisState = 0x07;
constexpr uint8_t kCmdSetAxisNodeId = 0x06;
constexpr uint8_t kCmdSetCtrlMode = 0x0B;
constexpr uint8_t kCmdSetInputPos = 0x0C;
constexpr uint8_t kCmdGetEncoder = 0x09;
constexpr uint8_t kCmdSaveConfig = 0x1F;

uint32_t make_id(uint8_t node_id, uint8_t cmd_id) {
    return (static_cast<uint32_t>(node_id) << 5) | (cmd_id & 0x1FU);
}

void reset_fakes(void) {
    g_fdcan_tx_count = 0;
    g_gpio_left_upper_state = GPIO_PIN_RESET;
    g_gpio_left_lower_state = GPIO_PIN_RESET;
    g_gpio_right_upper_state = GPIO_PIN_RESET;
    g_gpio_right_lower_state = GPIO_PIN_RESET;
}

void pack_f32(uint8_t *dst, float value) {
    std::memcpy(dst, &value, sizeof(value));
}

void send_encoder(uint8_t node_id, float theta_rad, float vel_rev_s, uint32_t now_ms) {
    uint8_t payload[8] = {0};
    float pos_rev = theta_rad / (2.0f * kPi);
    pack_f32(&payload[0], pos_rev);
    pack_f32(&payload[4], vel_rev_s);
    hip_control_on_can_rx(make_id(node_id, kCmdGetEncoder), payload, 8U, now_ms);
}

float pos_rev_from_theta(float theta_rad)
{
    return theta_rad / (2.0f * kPi);
}

float theta_from_pos_rev(float pos_rev)
{
    return pos_rev * (2.0f * kPi);
}

bool has_cmd_since(uint32_t start_idx, uint8_t node_id, uint8_t cmd_id) {
    for (uint32_t i = start_idx; i < g_fdcan_tx_count && i < 16U; ++i) {
        uint32_t id = g_fdcan_tx_headers[i].Identifier;
        uint8_t node = static_cast<uint8_t>((id >> 5) & 0x3FU);
        uint8_t cmd = static_cast<uint8_t>(id & 0x1FU);
        if (node == node_id && cmd == cmd_id) {
            return true;
        }
    }
    return false;
}

bool get_last_input_pos(uint8_t node_id, float *pos_rev, float *vel_ff_rev_s, float *torque_ff_nm)
{
    if (g_fdcan_tx_count == 0U) {
        return false;
    }
    for (int32_t i = (int32_t)g_fdcan_tx_count - 1; i >= 0; --i) {
        uint32_t id = g_fdcan_tx_headers[i].Identifier;
        uint8_t node = static_cast<uint8_t>((id >> 5) & 0x3FU);
        uint8_t cmd = static_cast<uint8_t>(id & 0x1FU);
        if (node == node_id && cmd == kCmdSetInputPos) {
            if (pos_rev) {
                float pos = 0.0f;
                std::memcpy(&pos, &g_fdcan_tx_data[i][0], sizeof(pos));
                *pos_rev = pos;
            }
            if (vel_ff_rev_s) {
                int16_t vel_ff = 0;
                std::memcpy(&vel_ff, &g_fdcan_tx_data[i][4], sizeof(vel_ff));
                *vel_ff_rev_s = static_cast<float>(vel_ff) / 1000.0f;
            }
            if (torque_ff_nm) {
                int16_t torque_ff = 0;
                std::memcpy(&torque_ff, &g_fdcan_tx_data[i][6], sizeof(torque_ff));
                *torque_ff_nm = static_cast<float>(torque_ff) / 1000.0f;
            }
            return true;
        }
    }
    return false;
}

}  // namespace

TEST_CASE("hip_control programs CAN Simple node id and optional save", "[hip][programming]")
{
    reset_fakes();

    REQUIRE(hip_control_program_node_id(0U, 3U, true));
    REQUIRE(g_fdcan_tx_count == 2U);

    CHECK(g_fdcan_tx_headers[0].Identifier == make_id(0U, kCmdSetAxisNodeId));
    uint32_t programmed_id = 0U;
    std::memcpy(&programmed_id, g_fdcan_tx_data[0], sizeof(programmed_id));
    CHECK(programmed_id == 3U);

    CHECK(g_fdcan_tx_headers[1].Identifier == make_id(3U, kCmdSaveConfig));
    CHECK(g_fdcan_tx_headers[1].DataLength == FDCAN_DLC_BYTES_0);
}

TEST_CASE("hip_control rejects invalid CAN Simple node ids", "[hip][programming]")
{
    reset_fakes();

    CHECK_FALSE(hip_control_program_node_id(64U, 3U, false));
    CHECK_FALSE(hip_control_program_node_id(0U, 64U, false));
    CHECK(g_fdcan_tx_count == 0U);
}

TEST_CASE("hip_control startup sequence sends ctrl mode then axis state")
{
    reset_fakes();
    hip_control_init();

    hip_control_tick(0U);
    REQUIRE(g_fdcan_tx_count == 2U);
    CHECK(g_fdcan_tx_headers[0].Identifier == make_id(kNodeLeft, kCmdSetCtrlMode));
    CHECK(g_fdcan_tx_headers[1].Identifier == make_id(kNodeRight, kCmdSetCtrlMode));

    hip_control_tick(5U);
    CHECK(g_fdcan_tx_count == 2U);

    hip_control_tick(10U);
    REQUIRE(g_fdcan_tx_count == 4U);
    CHECK(g_fdcan_tx_headers[2].Identifier == make_id(kNodeLeft, kCmdSetAxisState));
    CHECK(g_fdcan_tx_headers[3].Identifier == make_id(kNodeRight, kCmdSetAxisState));
}

TEST_CASE("hip_control sends position commands after encoder updates")
{
    reset_fakes();
    hip_control_init();

    hip_control_tick(0U);
    hip_control_tick(10U);

    send_encoder(kNodeLeft, 0.7f, 0.0f, 12U);
    send_encoder(kNodeRight, 0.7f, 0.0f, 12U);

    uint32_t start_idx = g_fdcan_tx_count;
    hip_control_tick(20U);

    CHECK(has_cmd_since(start_idx, kNodeLeft, kCmdSetInputPos));
    CHECK(has_cmd_since(start_idx, kNodeRight, kCmdSetInputPos));
}

TEST_CASE("hip_control bus-off prevents new position commands")
{
    reset_fakes();
    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    send_encoder(kNodeLeft, 0.7f, 0.0f, 12U);
    send_encoder(kNodeRight, 0.7f, 0.0f, 12U);

    hip_control_tick(20U);
    uint32_t start_idx = g_fdcan_tx_count;

    hip_control_on_bus_off();
    hip_control_tick(30U);

    CHECK_FALSE(has_cmd_since(start_idx, kNodeLeft, kCmdSetInputPos));
    CHECK_FALSE(has_cmd_since(start_idx, kNodeRight, kCmdSetInputPos));
}

TEST_CASE("hip_control bus-off recovery restarts startup sequence", "[hip][can]")
{
    reset_fakes();
    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    hip_control_on_bus_off();
    uint32_t start_idx = g_fdcan_tx_count;
    hip_control_tick(30U);
    CHECK_FALSE(has_cmd_since(start_idx, kNodeLeft, kCmdSetInputPos));

    hip_control_on_bus_recovered();
    hip_control_tick(40U);
    hip_control_tick(50U);
    CHECK(has_cmd_since(start_idx, kNodeLeft, kCmdSetCtrlMode));
    CHECK(has_cmd_since(start_idx, kNodeRight, kCmdSetCtrlMode));
}

TEST_CASE("hip_control telemetry timeout disables commands", "[hip][safety]")
{
    reset_fakes();
    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    send_encoder(kNodeLeft, 0.7f, 0.0f, 12U);
    send_encoder(kNodeRight, 0.7f, 0.0f, 12U);
    hip_control_on_can_rx(make_id(kNodeLeft, 0x01), nullptr, 0U, 12U);
    hip_control_on_can_rx(make_id(kNodeRight, 0x01), nullptr, 0U, 12U);

    hip_target_t target = {};
    target.height_ref_m = 0.6f;
    target.height_rate_ref_m_s = 0.0f;
    target.stiffness_n_m = 800.0f;
    target.damping_n_s_m = 80.0f;
    target.mode = HIP_MODE_HOLD;
    target.enabled = true;
    hip_control_set_target(&target);

    hip_control_tick(20U);
    uint32_t start_idx = g_fdcan_tx_count;

    hip_control_tick(HIP_TELEM_TIMEOUT_MS + 30U);
    uint32_t start_after_fault = g_fdcan_tx_count;
    hip_control_tick(HIP_TELEM_TIMEOUT_MS + 40U);
    CHECK_FALSE(has_cmd_since(start_after_fault, kNodeLeft, kCmdSetInputPos));
    CHECK_FALSE(has_cmd_since(start_after_fault, kNodeRight, kCmdSetInputPos));
}

TEST_CASE("hip_control heartbeat timeout raises fault", "[hip][safety]")
{
    reset_fakes();
    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    hip_control_on_can_rx(make_id(kNodeLeft, 0x01), nullptr, 0U, 10U);
    hip_control_on_can_rx(make_id(kNodeRight, 0x01), nullptr, 0U, 10U);

    hip_control_tick(HIP_TELEM_TIMEOUT_MS + 20U);
    CHECK((hip_control_get_faults() & HIP_FAULT_HEARTBEAT_TIMEOUT) != 0U);
}

TEST_CASE("hip_control stall fault triggers when command not followed", "[hip][safety]")
{
    reset_fakes();
    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    float theta_low = 0.0f;
    REQUIRE(hip_kinematics_theta_from_height(0.3f, &theta_low));
    send_encoder(kNodeLeft, theta_low, 0.0f, 10U);
    send_encoder(kNodeRight, theta_low, 0.0f, 10U);

    hip_target_t target = {};
    target.height_ref_m = 0.65f;
    target.height_rate_ref_m_s = 0.0f;
    target.stiffness_n_m = 800.0f;
    target.damping_n_s_m = 80.0f;
    target.mode = HIP_MODE_HOLD;
    target.enabled = true;
    hip_control_set_target(&target);

    hip_control_tick(20U);
    hip_command_t cmd_left{};
    hip_command_t cmd_right{};
    hip_control_get_command(&cmd_left, &cmd_right);
    float pos_current = pos_rev_from_theta(theta_low);
    REQUIRE(std::fabs(cmd_left.pos_cmd_rev - pos_current) > HIP_STALL_ERR_REV);

    send_encoder(kNodeLeft, theta_low, 0.0f, 20U);
    send_encoder(kNodeRight, theta_low, 0.0f, 20U);

    send_encoder(kNodeLeft, theta_low, 0.0f, 200U);
    send_encoder(kNodeRight, theta_low, 0.0f, 200U);
    send_encoder(kNodeLeft, theta_low, 0.0f, 300U);
    send_encoder(kNodeRight, theta_low, 0.0f, 300U);
    send_encoder(kNodeLeft, theta_low, 0.0f, 400U);
    send_encoder(kNodeRight, theta_low, 0.0f, 400U);

    hip_control_tick(HIP_STALL_TIMEOUT_MS + 30U);
    uint32_t faults = hip_control_get_faults();
    CHECK((faults & HIP_FAULT_STALL_LEFT) != 0U);
    CHECK((faults & HIP_FAULT_STALL_RIGHT) != 0U);
}

TEST_CASE("hip_control stall fault can be per-motor", "[hip][safety]")
{
    reset_fakes();
    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    float theta_low = 0.0f;
    REQUIRE(hip_kinematics_theta_from_height(0.3f, &theta_low));
    send_encoder(kNodeLeft, theta_low, 0.0f, 10U);
    send_encoder(kNodeRight, theta_low, 0.0f, 10U);

    hip_target_t target = {};
    target.height_ref_m = 0.65f;
    target.height_rate_ref_m_s = 0.0f;
    target.stiffness_n_m = 800.0f;
    target.damping_n_s_m = 80.0f;
    target.mode = HIP_MODE_HOLD;
    target.enabled = true;
    hip_control_set_target(&target);

    hip_control_tick(20U);
    hip_command_t cmd_left{};
    hip_command_t cmd_right{};
    hip_control_get_command(&cmd_left, &cmd_right);

    send_encoder(kNodeLeft, theta_low, 0.0f, 20U);
    send_encoder(kNodeRight, theta_from_pos_rev(cmd_right.pos_cmd_rev), 0.0f, 20U);

    send_encoder(kNodeLeft, theta_low, 0.0f, 300U);
    send_encoder(kNodeRight, theta_from_pos_rev(cmd_right.pos_cmd_rev), 0.0f, 300U);
    send_encoder(kNodeLeft, theta_low, 0.0f, 520U);
    send_encoder(kNodeRight, theta_from_pos_rev(cmd_right.pos_cmd_rev), 0.0f, 520U);

    hip_control_tick(HIP_STALL_TIMEOUT_MS + 30U);
    uint32_t faults = hip_control_get_faults();
    CHECK((faults & HIP_FAULT_STALL_LEFT) != 0U);
    CHECK((faults & HIP_FAULT_STALL_RIGHT) == 0U);
}

TEST_CASE("hip_control stall fault clears when motion resumes", "[hip][safety]")
{
    reset_fakes();
    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    float theta_low = 0.0f;
    REQUIRE(hip_kinematics_theta_from_height(0.3f, &theta_low));
    send_encoder(kNodeLeft, theta_low, 0.0f, 10U);
    send_encoder(kNodeRight, theta_low, 0.0f, 10U);

    hip_target_t target = {};
    target.height_ref_m = 0.65f;
    target.height_rate_ref_m_s = 0.0f;
    target.stiffness_n_m = 800.0f;
    target.damping_n_s_m = 80.0f;
    target.mode = HIP_MODE_HOLD;
    target.enabled = true;
    hip_control_set_target(&target);

    hip_control_tick(20U);
    hip_command_t cmd_left{};
    hip_command_t cmd_right{};
    hip_control_get_command(&cmd_left, &cmd_right);

    send_encoder(kNodeLeft, theta_low, 0.0f, 20U);
    send_encoder(kNodeRight, theta_low, 0.0f, 20U);
    send_encoder(kNodeLeft, theta_low, 0.0f, 300U);
    send_encoder(kNodeRight, theta_low, 0.0f, 300U);
    send_encoder(kNodeLeft, theta_low, 0.0f, 520U);
    send_encoder(kNodeRight, theta_low, 0.0f, 520U);
    hip_control_tick(HIP_STALL_TIMEOUT_MS + 30U);
    CHECK((hip_control_get_faults() & HIP_FAULT_STALL) != 0U);

    send_encoder(kNodeLeft, theta_from_pos_rev(cmd_left.pos_cmd_rev), 0.0f, 600U);
    send_encoder(kNodeRight, theta_from_pos_rev(cmd_right.pos_cmd_rev), 0.0f, 600U);
    hip_control_tick(610U);
    CHECK((hip_control_get_faults() & HIP_FAULT_STALL) == 0U);
}

TEST_CASE("hip_control concurrent faults include bus-off and encoder timeout", "[hip][safety]")
{
    reset_fakes();
    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    send_encoder(kNodeLeft, 0.5f, 0.0f, 10U);
    send_encoder(kNodeRight, 0.5f, 0.0f, 10U);
    hip_control_on_bus_off();
    hip_control_tick(HIP_TELEM_TIMEOUT_MS + 30U);

    uint32_t faults = hip_control_get_faults();
    CHECK((faults & HIP_FAULT_BUS_OFF) != 0U);
    CHECK((faults & HIP_FAULT_ENCODER_TIMEOUT) != 0U);
}

TEST_CASE("hip_control recovery mode engages after debounced limit", "[hip][limits]")
{
    reset_fakes();
    g_gpio_left_upper_state = GPIO_PIN_SET;

    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    float theta_mid = 0.0f;
    REQUIRE(hip_kinematics_theta_from_height(0.55f, &theta_mid));
    send_encoder(kNodeLeft, theta_mid, 0.0f, 12U);
    send_encoder(kNodeRight, theta_mid, 0.0f, 12U);

    for (uint32_t t = 12U; t < 12U + HIP_LIMIT_DEBOUNCE_SAMPLES; ++t) {
        hip_control_tick(t);
    }

    hip_target_t target = {};
    target.height_ref_m = 0.7f;
    target.height_rate_ref_m_s = 0.0f;
    target.stiffness_n_m = 800.0f;
    target.damping_n_s_m = 80.0f;
    target.mode = HIP_MODE_HOLD;
    target.enabled = true;
    hip_control_set_target(&target);
    hip_control_tick(40U);

    hip_command_t cmd_left{};
    hip_control_get_command(&cmd_left, nullptr);
    CHECK(cmd_left.pos_cmd_rev < pos_rev_from_theta(theta_mid));
}

TEST_CASE("hip_control recovery stops after max travel without opposite limit", "[hip][limits]")
{
    reset_fakes();
    g_gpio_left_upper_state = GPIO_PIN_SET;

    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    float theta_start = 0.0f;
    REQUIRE(hip_kinematics_theta_from_height(0.55f, &theta_start));
    send_encoder(kNodeLeft, theta_start, 0.0f, 12U);
    send_encoder(kNodeRight, theta_start, 0.0f, 12U);

    for (uint32_t t = 12U; t < 12U + HIP_LIMIT_DEBOUNCE_SAMPLES; ++t) {
        hip_control_tick(t);
    }

    send_encoder(kNodeLeft, theta_start, 0.0f, 100U);
    send_encoder(kNodeRight, theta_start, 0.0f, 100U);

    float theta_range = (HIP_THETA_MAX_DEG - HIP_THETA_MIN_DEG) * (kPi / 180.0f);
    float theta_far = theta_start + theta_range + 0.05f;
    send_encoder(kNodeLeft, theta_far, 0.0f, 200U);
    send_encoder(kNodeRight, theta_far, 0.0f, 200U);

    hip_target_t target = {};
    target.height_ref_m = 0.7f;
    target.height_rate_ref_m_s = 0.0f;
    target.stiffness_n_m = 800.0f;
    target.damping_n_s_m = 80.0f;
    target.mode = HIP_MODE_HOLD;
    target.enabled = true;
    hip_control_set_target(&target);
    hip_control_tick(210U);

    hip_command_t cmd_left{};
    hip_control_get_command(&cmd_left, nullptr);
    CHECK(cmd_left.pos_cmd_rev <= pos_rev_from_theta(theta_start) + 1.0e-4f);
}

TEST_CASE("hip_control recovery exits when limit clears", "[hip][limits]")
{
    reset_fakes();
    g_gpio_left_upper_state = GPIO_PIN_SET;

    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    float theta_mid = 0.0f;
    REQUIRE(hip_kinematics_theta_from_height(0.55f, &theta_mid));
    send_encoder(kNodeLeft, theta_mid, 0.0f, 12U);
    send_encoder(kNodeRight, theta_mid, 0.0f, 12U);

    for (uint32_t t = 12U; t < 12U + HIP_LIMIT_DEBOUNCE_SAMPLES; ++t) {
        hip_control_tick(t);
    }

    g_gpio_left_upper_state = GPIO_PIN_RESET;
    for (uint32_t t = 20U; t < 20U + HIP_LIMIT_DEBOUNCE_SAMPLES; ++t) {
        hip_control_tick(t);
    }

    hip_target_t target = {};
    target.height_ref_m = 0.4f;
    target.height_rate_ref_m_s = 0.0f;
    target.stiffness_n_m = 800.0f;
    target.damping_n_s_m = 80.0f;
    target.mode = HIP_MODE_HOLD;
    target.enabled = true;
    hip_control_set_target(&target);
    hip_control_tick(40U);

    hip_command_t cmd_left{};
    hip_control_get_command(&cmd_left, nullptr);
    CHECK(cmd_left.pos_cmd_rev < pos_rev_from_theta(theta_mid));
}

TEST_CASE("hip_control recovery timeout releases to target", "[hip][limits]")
{
    reset_fakes();
    g_gpio_left_upper_state = GPIO_PIN_SET;

    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    float theta_mid = 0.0f;
    REQUIRE(hip_kinematics_theta_from_height(0.55f, &theta_mid));
    send_encoder(kNodeLeft, theta_mid, 0.0f, 12U);
    send_encoder(kNodeRight, theta_mid, 0.0f, 12U);

    for (uint32_t t = 12U; t < 12U + HIP_LIMIT_DEBOUNCE_SAMPLES; ++t) {
        hip_control_tick(t);
    }

    hip_target_t target = {};
    target.height_ref_m = 0.3f;
    target.height_rate_ref_m_s = 0.0f;
    target.stiffness_n_m = 800.0f;
    target.damping_n_s_m = 80.0f;
    target.mode = HIP_MODE_HOLD;
    target.enabled = true;
    hip_control_set_target(&target);

    hip_control_tick(100U);
    hip_command_t cmd_before{};
    hip_control_get_command(&cmd_before, nullptr);

    hip_control_tick(HIP_RECOVERY_TIMEOUT_MS + 200U);
    hip_command_t cmd_after{};
    hip_control_get_command(&cmd_after, nullptr);

    CHECK(cmd_after.pos_cmd_rev < cmd_before.pos_cmd_rev);
}

TEST_CASE("hip_control clamps target height to kinematic range", "[hip][limits]")
{
    reset_fakes();
    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    float h_min = 0.0f;
    float h_max = 0.0f;
    REQUIRE(hip_kinematics_get_height_range(&h_min, &h_max));
    float theta_mid = 0.0f;
    REQUIRE(hip_kinematics_theta_from_height(0.5f, &theta_mid));

    send_encoder(kNodeLeft, theta_mid, 0.0f, 12U);
    send_encoder(kNodeRight, theta_mid, 0.0f, 12U);

    hip_target_t target = {};
    target.height_ref_m = h_max + 0.1f;
    target.height_rate_ref_m_s = 0.0f;
    target.stiffness_n_m = 800.0f;
    target.damping_n_s_m = 80.0f;
    target.mode = HIP_MODE_HOLD;
    target.enabled = true;
    hip_control_set_target(&target);

    hip_control_tick(20U);

    hip_command_t cmd_left{};
    hip_command_t cmd_right{};
    hip_control_get_command(&cmd_left, &cmd_right);

    float theta_cmd = theta_from_pos_rev(cmd_left.pos_cmd_rev);
    float h_cmd = 0.0f;
    REQUIRE(hip_kinematics_height_from_theta(theta_cmd, &h_cmd));
    CHECK(h_cmd <= h_max + 1.0e-3f);
}

TEST_CASE("hip_control slew limits height reference step", "[hip][slew]")
{
    reset_fakes();
    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    float theta_mid = 0.0f;
    REQUIRE(hip_kinematics_theta_from_height(0.5f, &theta_mid));
    send_encoder(kNodeLeft, theta_mid, 0.0f, 12U);
    send_encoder(kNodeRight, theta_mid, 0.0f, 12U);

    hip_target_t target = {};
    target.height_ref_m = 0.5f;
    target.height_rate_ref_m_s = 0.0f;
    target.stiffness_n_m = 800.0f;
    target.damping_n_s_m = 80.0f;
    target.mode = HIP_MODE_HOLD;
    target.enabled = true;
    hip_control_set_target(&target);
    hip_control_tick(20U);

    target.height_ref_m = 0.6f;
    hip_control_set_target(&target);
    hip_control_tick(30U);

    hip_command_t cmd_left{};
    hip_command_t cmd_right{};
    hip_control_get_command(&cmd_left, &cmd_right);

    float theta_cmd = theta_from_pos_rev(cmd_left.pos_cmd_rev);
    float h_cmd = 0.0f;
    REQUIRE(hip_kinematics_height_from_theta(theta_cmd, &h_cmd));
    float expected = 0.5f + HIP_HEIGHT_SLEW_MPS * 0.01f;
    CHECK(h_cmd == Catch::Approx(expected).margin(0.01f));
}

TEST_CASE("hip_control limit prevents moving further into limit", "[hip][limits]")
{
    reset_fakes();
    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    g_gpio_left_upper_state = GPIO_PIN_SET;

    float theta_max = 0.0f;
    REQUIRE(hip_kinematics_theta_from_height(0.65f, &theta_max));
    send_encoder(kNodeLeft, theta_max, 0.0f, 12U);
    send_encoder(kNodeRight, theta_max, 0.0f, 12U);

    hip_target_t target = {};
    target.height_ref_m = 0.7f;
    target.height_rate_ref_m_s = 0.0f;
    target.stiffness_n_m = 800.0f;
    target.damping_n_s_m = 80.0f;
    target.mode = HIP_MODE_HOLD;
    target.enabled = true;
    hip_control_set_target(&target);

    hip_control_tick(20U);
    hip_control_tick(21U);
    hip_control_tick(22U);
    hip_control_tick(23U);
    hip_control_tick(30U);

    hip_command_t cmd_left{};
    hip_command_t cmd_right{};
    hip_control_get_command(&cmd_left, &cmd_right);

    float pos_current = pos_rev_from_theta(theta_max);
    CHECK(cmd_left.pos_cmd_rev == Catch::Approx(pos_current).margin(0.005f));
}

TEST_CASE("hip_control apply_params flips command direction", "[hip][config]")
{
    reset_fakes();
    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    float theta_mid = 0.0f;
    REQUIRE(hip_kinematics_theta_from_height(0.55f, &theta_mid));
    float pos_rev = pos_rev_from_theta(theta_mid);

    robot_params_t params = {};
    params.hip_left_dir_sign = -1;
    params.hip_right_dir_sign = -1;
    hip_control_apply_params(&params);

    send_encoder(kNodeLeft, -theta_mid, 0.0f, 12U);
    send_encoder(kNodeRight, -theta_mid, 0.0f, 12U);

    hip_target_t target = {};
    target.height_ref_m = 0.55f;
    target.height_rate_ref_m_s = 0.0f;
    target.stiffness_n_m = 800.0f;
    target.damping_n_s_m = 80.0f;
    target.mode = HIP_MODE_HOLD;
    target.enabled = true;
    hip_control_set_target(&target);
    hip_control_tick(20U);

    hip_command_t cmd_left{};
    hip_command_t cmd_right{};
    hip_control_get_command(&cmd_left, &cmd_right);

    CHECK(cmd_left.pos_cmd_rev == Catch::Approx(-pos_rev).margin(0.005f));
    CHECK(cmd_right.pos_cmd_rev == Catch::Approx(-pos_rev).margin(0.005f));
}

TEST_CASE("hip_control applies large zero offsets", "[hip][config]")
{
    reset_fakes();
    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    float theta_mid = 0.0f;
    REQUIRE(hip_kinematics_theta_from_height(0.55f, &theta_mid));
    float pos_rev = pos_rev_from_theta(theta_mid);

    robot_params_t params = {};
    params.hip_left_zero_offset_rev = 5.0f;
    params.hip_right_zero_offset_rev = -4.0f;
    hip_control_apply_params(&params);

    send_encoder(kNodeLeft, theta_mid, 0.0f, 12U);
    send_encoder(kNodeRight, theta_mid, 0.0f, 12U);

    hip_target_t target = {};
    target.height_ref_m = 0.55f;
    target.height_rate_ref_m_s = 0.0f;
    target.stiffness_n_m = 800.0f;
    target.damping_n_s_m = 80.0f;
    target.mode = HIP_MODE_HOLD;
    target.enabled = true;
    hip_control_set_target(&target);
    hip_control_tick(20U);

    hip_command_t cmd_left{};
    hip_command_t cmd_right{};
    hip_control_get_command(&cmd_left, &cmd_right);

    CHECK(cmd_left.pos_cmd_rev == Catch::Approx(pos_rev + 5.0f).margin(0.01f));
    CHECK(cmd_right.pos_cmd_rev == Catch::Approx(pos_rev - 4.0f).margin(0.01f));
}

TEST_CASE("hip_control clamps velocity and torque feedforward", "[hip][limits]")
{
    reset_fakes();
    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    float theta_low = 0.0f;
    REQUIRE(hip_kinematics_theta_from_height(0.4f, &theta_low));
    send_encoder(kNodeLeft, theta_low, 0.0f, 12U);
    send_encoder(kNodeRight, theta_low, 0.0f, 12U);

    hip_target_t target = {};
    target.height_ref_m = 0.7f;
    target.height_rate_ref_m_s = 2.0f;
    target.stiffness_n_m = 1200.0f;
    target.damping_n_s_m = 200.0f;
    target.mode = HIP_MODE_HOLD;
    target.enabled = true;
    hip_control_set_target(&target);

    hip_control_tick(20U);

    hip_command_t cmd_left{};
    hip_command_t cmd_right{};
    hip_control_get_command(&cmd_left, &cmd_right);

    CHECK(std::fabs(cmd_left.vel_ff_rev_s) <= HIP_VEL_MAX_REV_S + 1.0e-3f);
    CHECK(std::fabs(cmd_left.torque_ff_nm) <= HIP_TORQUE_MAX_NM + 1.0e-3f);
    CHECK(std::fabs(cmd_right.vel_ff_rev_s) <= HIP_VEL_MAX_REV_S + 1.0e-3f);
    CHECK(std::fabs(cmd_right.torque_ff_nm) <= HIP_TORQUE_MAX_NM + 1.0e-3f);
}
