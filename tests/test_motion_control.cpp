#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <cstring>

extern "C" {
#include "motion_control.h"
#include "motion_modes.h"
#include "hip_behavior.h"
#include "hip_control.h"
#include "app_config.h"
#include "app_main.h"
}

#include "param_storage.h"
#include "imu_bmi270.h"

extern "C" {
void imu_fake_set_bmi_sample(const imu_bmi270_sample_t *sample, uint32_t irq_ms);
void motor_link_fake_set_velocities(float left_rad_s, float right_rad_s);
void motor_link_fake_get_last_torque(float *left_nm, float *right_nm, float *max_nm);
}

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr uint8_t kHipCmdGetEncoder = 0x09U;

void reset_params()
{
    g_robot_params = {};
    g_robot_params.imu_bmi270.gyro_bias[0] = 1;
    g_robot_params.imu_icm42688.gyro_bias[0] = 1;
}

void setup_balance_params()
{
    reset_params();
    g_robot_params.wheel_radius_m = 0.06f;
    g_robot_params.wheel_base_m = 0.30f;
    g_robot_params.max_linear_vel_mps = 1.0f;
    g_robot_params.max_angular_vel_rps = 2.0f;
    g_robot_params.control_rate_hz = 100.0f;

    g_robot_params.balance.Kp_theta = 20.0f;
    g_robot_params.balance.Kd_theta = 0.5f;
    g_robot_params.balance.Kp_v_to_theta = 1.0f;
    g_robot_params.balance.Ki_v_to_theta = 0.0f;
    g_robot_params.balance.max_tilt_ref = 0.5f;
    g_robot_params.balance.Kv_damp = 0.0f;
    g_robot_params.balance.K_turn = 0.0f;
    g_robot_params.balance.K_yawDamp = 0.0f;
    g_robot_params.balance.alpha_yaw = 0.0f;
    g_robot_params.balance.IqMax = 10.0f;
    g_robot_params.balance.thetaKill = 1.0f;
    g_robot_params.balance.iV_max = 0.5f;
}

void prime_imu(uint32_t now_ms)
{
    imu_bmi270_sample_t sample{};
    sample.accel[0] = 0;
    sample.accel[1] = 0;
    sample.accel[2] = 1000;
    sample.gyro[0] = 0;
    sample.gyro[1] = 0;
    sample.gyro[2] = 0;
    sample.timestamp_ms = now_ms;
    imu_fake_set_bmi_sample(&sample, now_ms);
}

float rev_from_rad(float theta_rad)
{
    return theta_rad / (2.0f * kPi);
}

void feed_hip_encoder(float left_theta_rad, float right_theta_rad, uint32_t now_ms)
{
    auto feed_one = [now_ms](uint8_t node_id, float theta_rad) {
        uint8_t payload[8] = {0};
        const float pos_rev = rev_from_rad(theta_rad);
        const float vel_rev_s = 0.0f;
        std::memcpy(&payload[0], &pos_rev, sizeof(pos_rev));
        std::memcpy(&payload[4], &vel_rev_s, sizeof(vel_rev_s));
        const uint32_t id = ((uint32_t)node_id << 5) | kHipCmdGetEncoder;
        (void)hip_control_on_can_rx(id, payload, sizeof(payload), now_ms);
    };

    feed_one(HIP_CAN_NODE_LEFT, left_theta_rad);
    feed_one(HIP_CAN_NODE_RIGHT, right_theta_rad);
}
} // namespace

TEST_CASE("motion_control_is_calibrated requires IMU biases", "[motion_control]")
{
    g_robot_params = {};
    CHECK_FALSE(motion_control_is_calibrated());

    reset_params();
    CHECK(motion_control_is_calibrated());
}

TEST_CASE("motion_control_can_arm gates on estimate, imu health, and motor link", "[motion_control]")
{
    reset_params();
    motion_modes_set(MOTION_MODE_DISARMED);

    motion_control_test_clear_overrides();

    motion_control_estimate_t est{};
    est.theta_rad = 0.0f;
    est.valid = 1U;
    motion_control_test_set_estimate(&est);

    motion_control_imu_health_t health{};
    health.valid = 1U;
    motion_control_test_set_imu_health(&health);

    CHECK(motion_control_can_arm());

    est.valid = 0U;
    motion_control_test_set_estimate(&est);
    CHECK_FALSE(motion_control_can_arm());
}

TEST_CASE("motion_control_get_imu_health reports override", "[motion_control]")
{
    motion_control_test_clear_overrides();
    motion_control_imu_health_t health_in{};
    health_in.valid = 1U;
    health_in.active_sensor = 1U;
    health_in.gyro_diff_dps = 2.5f;
    health_in.gyro_pitch_diff_dps = 1.2f;
    health_in.acc_angle_diff_deg = 0.5f;
    health_in.vib_rms_g = 0.01f;
    health_in.gate_accel = 0U;
    motion_control_test_set_imu_health(&health_in);

    motion_control_imu_health_t health_out{};
    REQUIRE(motion_control_get_imu_health(&health_out));
    CHECK(health_out.valid == 1U);
    CHECK(health_out.active_sensor == 1U);
    CHECK(health_out.gyro_diff_dps == Catch::Approx(2.5f));
}

TEST_CASE("motion_control wires hip behavior target into hip_control", "[motion_control][hip]")
{
    setup_balance_params();
    motion_control_init();
    motion_control_apply_params();
    motion_control_set_mode(MOTION_MODE_BALANCING);

    prime_imu(0U);
    motion_control_tick(0U);

    hip_target_t target{};
    hip_control_get_target(&target);
    CHECK(target.enabled);
    CHECK(target.mode == HIP_MODE_HOLD);
}

TEST_CASE("motion_control scales wheel iq during jump phases", "[motion_control][hip]")
{
    setup_balance_params();
    motion_control_init();
    motion_control_apply_params();
    motion_control_set_mode(MOTION_MODE_BALANCING);
    motion_control_test_clear_overrides();

    motion_control_estimate_t est{};
    est.theta_rad = 0.15f;
    est.theta_dot = 0.0f;
    est.x_m = 0.0f;
    est.x_dot_mps = 0.0f;
    est.gyro_bias = 0.0f;
    est.valid = 1U;
    motion_control_test_set_estimate(&est);

    motor_link_fake_set_velocities(0.0f, 0.0f);
    prime_imu(0U);
    hip_behavior_request_jump();

    motion_control_tick(0U);
    motion_control_tick(20U);
    float torque_crouch = 0.0f;
    motor_link_fake_get_last_torque(&torque_crouch, nullptr, nullptr);

    motion_control_tick(HIP_BEHAVIOR_CROUCH_MS + 5U);
    motion_control_tick(HIP_BEHAVIOR_CROUCH_MS + 25U);
    float torque_impulse = 0.0f;
    motor_link_fake_get_last_torque(&torque_impulse, nullptr, nullptr);

    motion_control_tick(HIP_BEHAVIOR_CROUCH_MS + HIP_BEHAVIOR_IMPULSE_MS + 5U);
    motion_control_tick(HIP_BEHAVIOR_CROUCH_MS + HIP_BEHAVIOR_IMPULSE_MS + 25U);
    float torque_flight = 0.0f;
    motor_link_fake_get_last_torque(&torque_flight, nullptr, nullptr);

    motion_control_tick(HIP_BEHAVIOR_CROUCH_MS + HIP_BEHAVIOR_IMPULSE_MS +
                        HIP_BEHAVIOR_FLIGHT_MS + 5U);
    motion_control_tick(HIP_BEHAVIOR_CROUCH_MS + HIP_BEHAVIOR_IMPULSE_MS +
                        HIP_BEHAVIOR_FLIGHT_MS + 25U);
    float torque_landing = 0.0f;
    motor_link_fake_get_last_torque(&torque_landing, nullptr, nullptr);

    REQUIRE(torque_crouch != 0.0f);
    CHECK(torque_impulse == Catch::Approx(torque_crouch * HIP_WHEEL_SCALE_IMPULSE).margin(0.05f));
    CHECK(torque_flight == Catch::Approx(torque_crouch * HIP_WHEEL_SCALE_FLIGHT).margin(0.05f));
    CHECK(torque_landing == Catch::Approx(torque_crouch * HIP_WHEEL_SCALE_LANDING).margin(0.05f));
}

TEST_CASE("motion_control applies LUT equilibrium to LQR output in tick path",
          "[motion_control][lqr][hip]")
{
    setup_balance_params();
    g_robot_params.lqr.default_mode = 1U;
    g_robot_params.lqr.engage_ramp_ms = 0U;
    g_robot_params.lqr.disengage_ramp_ms = 0U;
    g_robot_params.lqr.u_limit = 200.0f;
    g_robot_params.lqr.du_limit = 0.0f;
    g_robot_params.lqr.theta_ref_limit = 1.0f;
    g_robot_params.lqr.v_ref_limit = 2.0f;
    g_robot_params.lqr.K[0] = 0.0f;
    g_robot_params.lqr.K[1] = -35.0f;
    g_robot_params.lqr.K[2] = -400.0f;
    g_robot_params.lqr.K[3] = -24.0f;

    motion_control_init();
    motion_control_apply_params();
    motion_control_set_mode(MOTION_MODE_BALANCING);
    motion_control_test_clear_overrides();

    motion_control_estimate_t est{};
    est.theta_rad = 0.0f;
    est.theta_dot = 0.0f;
    est.x_m = 0.0f;
    est.x_dot_mps = 0.0f;
    est.gyro_bias = 0.0f;
    est.valid = 1U;
    motion_control_test_set_estimate(&est);
    motor_link_fake_set_velocities(0.0f, 0.0f);

    auto tick = [](uint32_t now_ms) {
        prime_imu(now_ms);
        motion_control_tick(now_ms);
    };

    /* Bring hip control startup to closed-loop stage first. */
    tick(0U);
    tick(10U);
    tick(20U);

    /* Low hip angle should map to negative theta_eq and positive wheel torque. */
    feed_hip_encoder(-0.27f, -0.27f, 25U);
    tick(40U);
    tick(60U);
    tick(80U);
    REQUIRE(motion_control_get_inner_mode() == 1U);
    float torque_low = 0.0f;
    motor_link_fake_get_last_torque(&torque_low, nullptr, nullptr);

    /* Higher hip angle should map to different theta_eq and shift output torque. */
    feed_hip_encoder(0.16f, 0.16f, 85U);
    tick(100U);
    tick(120U);
    tick(140U);
    float torque_high = 0.0f;
    motor_link_fake_get_last_torque(&torque_high, nullptr, nullptr);

    CHECK(std::fabs(torque_low - torque_high) > 10.0f);
}
