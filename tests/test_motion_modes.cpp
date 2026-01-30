#include <catch2/catch_test_macros.hpp>

extern "C" {
#include "motion_modes.h"
}

#include "config_control.h"
#include "param_storage.h"

robot_params_t g_robot_params = {
    .dump_seconds_default = 30,
};

namespace {
motion_modes_input_t make_base_input(uint32_t now_ms)
{
    motion_modes_input_t input = {};
    input.now_ms = now_ms;
    input.estimate_valid = true;
    input.theta_rad = 0.0f;
    input.theta_kill_rad = 0.3f;
    input.theta_acc_valid = true;
    input.theta_acc_rad = 0.0f;
    input.imu_ok = true;
    input.motor_ok = true;
    input.last_imu_ok_ms = now_ms;
    input.last_imu_irq_ms = now_ms;
    input.last_motor_ok_ms = now_ms;
    return input;
}
} // namespace

TEST_CASE("motion_modes_init starts disarmed", "[motion_modes]")
{
    motion_modes_init();
    CHECK(motion_modes_get() == MOTION_MODE_DISARMED);
    CHECK_FALSE(motion_modes_allows_output());
}

TEST_CASE("motion_modes_allows_output only in balancing", "[motion_modes]")
{
    motion_modes_set(MOTION_MODE_BALANCING);
    CHECK(motion_modes_allows_output());

    motion_modes_set(MOTION_MODE_FALLEN);
    CHECK_FALSE(motion_modes_allows_output());
}

TEST_CASE("balancing -> fallen on kill angle", "[motion_modes]")
{
    motion_modes_set(MOTION_MODE_BALANCING);

    motion_modes_input_t input = make_base_input(1000);
    input.theta_rad = input.theta_kill_rad + 0.01f;

    motion_modes_output_t out = {};
    motion_modes_step(&input, &out);

    CHECK(out.mode_changed);
    CHECK(out.new_mode == MOTION_MODE_FALLEN);
    CHECK(out.disable_motors);
    CHECK(out.reset_pid);
}

TEST_CASE("balancing -> fallen on IMU timeout", "[motion_modes]")
{
    motion_modes_set(MOTION_MODE_BALANCING);

    motion_modes_input_t input = make_base_input(1000);
    input.last_imu_ok_ms = 1000U - IMU_FAULT_FALLEN_MS - 1U;

    motion_modes_output_t out = {};
    motion_modes_step(&input, &out);

    CHECK(out.mode_changed);
    CHECK(out.new_mode == MOTION_MODE_FALLEN);
}

TEST_CASE("balancing -> fault on fatal IMU timeout", "[motion_modes]")
{
    motion_modes_set(MOTION_MODE_BALANCING);

    motion_modes_input_t input = make_base_input(1000);
    input.last_imu_ok_ms = 1000U - IMU_FAULT_FATAL_MS - 1U;

    motion_modes_output_t out = {};
    motion_modes_step(&input, &out);

    CHECK(out.mode_changed);
    CHECK(out.new_mode == MOTION_MODE_FAULT);
}

TEST_CASE("disarmed does not transition to fault", "[motion_modes]")
{
    motion_modes_set(MOTION_MODE_DISARMED);

    motion_modes_input_t input = make_base_input(1000);
    input.last_imu_ok_ms = 1000U - IMU_FAULT_FATAL_MS - 1U;

    motion_modes_output_t out = {};
    motion_modes_step(&input, &out);

    CHECK_FALSE(out.mode_changed);
    CHECK(motion_modes_get() == MOTION_MODE_DISARMED);
}
