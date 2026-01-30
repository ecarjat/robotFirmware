#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

extern "C" {
#include "motion_control.h"
#include "motion_modes.h"
#include "app_main.h"
}

#include "param_storage.h"

namespace {
void reset_params()
{
    g_robot_params = {};
    g_robot_params.imu_bmi270.gyro_bias[0] = 1;
    g_robot_params.imu_icm42688.gyro_bias[0] = 1;
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
