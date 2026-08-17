#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <cstring>

extern "C" {
#include "app_config.h"
#include "hip_behavior.h"
#include "hip_control.h"
#include "hip_kinematics.h"
#include "main.h"
#include "stm32h7xx_hal_fdcan.h"
}

namespace {
constexpr float kPi = 3.14159265358979323846f;

float pos_rev_from_theta(float theta_rad)
{
    return theta_rad / (2.0f * kPi);
}

float theta_from_pos_rev(float pos_rev)
{
    return pos_rev * (2.0f * kPi);
}

void send_encoder(uint8_t node_id, float theta_rad, float vel_rev_s, uint32_t now_ms)
{
    uint8_t payload[8] = {0};
    float pos_rev = pos_rev_from_theta(theta_rad);
    std::memcpy(&payload[0], &pos_rev, sizeof(pos_rev));
    std::memcpy(&payload[4], &vel_rev_s, sizeof(vel_rev_s));
    uint32_t id = (static_cast<uint32_t>(node_id) << 5) | 0x09U;
    hip_control_on_can_rx(id, payload, 8U, now_ms);
}

}  // namespace

TEST_CASE("hip integration: full jump sequence with simulated feedback", "[hip][integration]")
{
    hip_control_init();
    hip_behavior_init(0.55f);

    float theta_start = 0.0f;
    REQUIRE(hip_kinematics_theta_from_height(0.55f, &theta_start));
    send_encoder(HIP_CAN_NODE_LEFT, theta_start, 0.0f, 0U);
    send_encoder(HIP_CAN_NODE_RIGHT, theta_start, 0.0f, 0U);

    hip_behavior_request_jump();

    hip_behavior_mode_t mode = HIP_BEHAVIOR_NORMAL;
    for (uint32_t t = 0U; t <= 800U; t += 20U) {
        hip_state_t left{};
        hip_state_t right{};
        hip_control_get_state(&left, &right);

        if (t >= HIP_BEHAVIOR_CROUCH_MS + 10U) {
            hip_behavior_set_imu_accel(HIP_LIFTOFF_ACCEL_G - 0.1f, true);
        }
        if (t >= HIP_BEHAVIOR_CROUCH_MS + HIP_BEHAVIOR_IMPULSE_MS + 40U) {
            hip_behavior_set_imu_accel(HIP_LANDING_ACCEL_G + 0.2f, true);
        }

        hip_target_t target{};
        hip_behavior_tick(t, MOTION_MODE_BALANCING, &left, &right, &target, &mode);
        hip_control_set_target(&target);
        hip_control_tick(t);

        hip_command_t cmd_left{};
        hip_command_t cmd_right{};
        hip_control_get_command(&cmd_left, &cmd_right);
        send_encoder(HIP_CAN_NODE_LEFT, theta_from_pos_rev(cmd_left.pos_cmd_rev), 0.0f, t);
        send_encoder(HIP_CAN_NODE_RIGHT, theta_from_pos_rev(cmd_right.pos_cmd_rev), 0.0f, t);
    }

    CHECK(mode == HIP_BEHAVIOR_NORMAL);
}

TEST_CASE("hip integration: limit recovery with encoder feedback", "[hip][integration]")
{
    g_gpio_left_upper_state = GPIO_PIN_SET;
    hip_control_init();

    float theta_mid = 0.0f;
    REQUIRE(hip_kinematics_theta_from_height(0.55f, &theta_mid));
    send_encoder(HIP_CAN_NODE_LEFT, theta_mid, 0.0f, 10U);
    send_encoder(HIP_CAN_NODE_RIGHT, theta_mid, 0.0f, 10U);

    for (uint32_t t = 10U; t < 10U + HIP_LIMIT_DEBOUNCE_SAMPLES; ++t) {
        hip_control_tick(t);
    }

    hip_target_t target{};
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

    g_gpio_left_upper_state = GPIO_PIN_RESET;
    for (uint32_t t = 50U; t < 50U + HIP_LIMIT_DEBOUNCE_SAMPLES; ++t) {
        hip_control_tick(t);
    }
    hip_control_tick(70U);
    hip_control_get_command(&cmd_left, nullptr);
    CHECK(cmd_left.pos_cmd_rev < pos_rev_from_theta(theta_mid));
}

TEST_CASE("hip integration: CAN bus-off recovery", "[hip][integration]")
{
    hip_control_init();
    hip_control_tick(0U);
    hip_control_tick(10U);

    hip_control_on_bus_off();
    hip_control_tick(30U);

    hip_control_on_bus_recovered();
    hip_control_tick(40U);
    hip_control_tick(50U);

    bool saw_ctrl = false;
    for (uint32_t i = 0; i < g_fdcan_tx_count && i < 16U; ++i) {
        if ((g_fdcan_tx_headers[i].Identifier & 0x1FU) == 0x0BU) {
            saw_ctrl = true;
            break;
        }
    }
    CHECK(saw_ctrl);
}
