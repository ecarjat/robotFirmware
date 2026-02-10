#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

extern "C" {
#include "hip_behavior.h"
#include "app_config.h"
}

namespace {

hip_state_t make_state(float height_m)
{
    hip_state_t state = {0};
    state.valid = 1U;
    state.height_m = height_m;
    return state;
}

}  // namespace

TEST_CASE("hip_behavior default normal holds measured height", "[hip][behavior]") {
    hip_behavior_init(0.5f);
    hip_state_t left = make_state(0.6f);
    hip_state_t right = make_state(0.6f);
    hip_target_t target = {};
    hip_behavior_mode_t mode = HIP_BEHAVIOR_NORMAL;

    hip_behavior_tick(0U, MOTION_MODE_BALANCING, &left, &right, &target, &mode);

    CHECK(mode == HIP_BEHAVIOR_NORMAL);
    CHECK(target.mode == HIP_MODE_HOLD);
    CHECK(target.enabled);
    CHECK(target.height_ref_m == Catch::Approx(0.6f));
}

TEST_CASE("hip_behavior jump sequence advances through phases", "[hip][behavior]") {
    hip_behavior_init(0.5f);
    hip_state_t left = make_state(0.5f);
    hip_state_t right = make_state(0.5f);
    hip_target_t target = {};
    hip_behavior_mode_t mode = HIP_BEHAVIOR_NORMAL;

    hip_behavior_request_jump();
    hip_behavior_tick(0U, MOTION_MODE_BALANCING, &left, &right, &target, &mode);
    CHECK(mode == HIP_BEHAVIOR_CROUCH);

    hip_behavior_tick(HIP_BEHAVIOR_CROUCH_MS + 1U, MOTION_MODE_BALANCING,
                      &left, &right, &target, &mode);
    CHECK(mode == HIP_BEHAVIOR_IMPULSE);

    hip_behavior_tick(HIP_BEHAVIOR_CROUCH_MS + HIP_BEHAVIOR_IMPULSE_MS + 2U,
                      MOTION_MODE_BALANCING, &left, &right, &target, &mode);
    CHECK(mode == HIP_BEHAVIOR_FLIGHT);

    hip_behavior_tick(HIP_BEHAVIOR_CROUCH_MS + HIP_BEHAVIOR_IMPULSE_MS +
                          HIP_BEHAVIOR_FLIGHT_MS + 3U,
                      MOTION_MODE_BALANCING, &left, &right, &target, &mode);
    CHECK(mode == HIP_BEHAVIOR_LANDING);

    hip_behavior_tick(HIP_BEHAVIOR_CROUCH_MS + HIP_BEHAVIOR_IMPULSE_MS +
                          HIP_BEHAVIOR_FLIGHT_MS + HIP_BEHAVIOR_LANDING_MS + 4U,
                      MOTION_MODE_BALANCING, &left, &right, &target, &mode);
    CHECK(mode == HIP_BEHAVIOR_NORMAL);
}

TEST_CASE("hip_behavior disables output when not balancing", "[hip][behavior]") {
    hip_behavior_init(0.5f);
    hip_state_t left = make_state(0.5f);
    hip_state_t right = make_state(0.5f);
    hip_target_t target = {};
    hip_behavior_mode_t mode = HIP_BEHAVIOR_NORMAL;

    hip_behavior_request_jump();
    hip_behavior_tick(0U, MOTION_MODE_DISARMED, &left, &right, &target, &mode);

    CHECK(mode == HIP_BEHAVIOR_NORMAL);
    CHECK_FALSE(target.enabled);
}

TEST_CASE("hip_behavior height reference evolves during jump phases", "[hip][behavior]") {
    hip_behavior_init(0.6f);
    hip_state_t left = make_state(0.6f);
    hip_state_t right = make_state(0.6f);
    hip_target_t target = {};
    hip_behavior_mode_t mode = HIP_BEHAVIOR_NORMAL;

    hip_behavior_request_jump();
    hip_behavior_tick(0U, MOTION_MODE_BALANCING, &left, &right, &target, &mode);
    float crouch_start = target.height_ref_m;

    hip_behavior_tick(50U, MOTION_MODE_BALANCING, &left, &right, &target, &mode);
    hip_behavior_tick(100U, MOTION_MODE_BALANCING, &left, &right, &target, &mode);
    CHECK(target.height_ref_m > crouch_start);

    hip_behavior_tick(HIP_BEHAVIOR_CROUCH_MS + 10U, MOTION_MODE_BALANCING,
                      &left, &right, &target, &mode);
    hip_behavior_tick(HIP_BEHAVIOR_CROUCH_MS + 30U, MOTION_MODE_BALANCING,
                      &left, &right, &target, &mode);
    float impulse_start = target.height_ref_m;
    hip_behavior_tick(HIP_BEHAVIOR_CROUCH_MS + 50U, MOTION_MODE_BALANCING,
                      &left, &right, &target, &mode);
    CHECK(target.height_ref_m < impulse_start);
}

TEST_CASE("hip_behavior uses IMU accel for liftoff/landing", "[hip][behavior]") {
    hip_behavior_init(0.6f);
    hip_state_t left = make_state(0.6f);
    hip_state_t right = make_state(0.6f);
    hip_target_t target = {};
    hip_behavior_mode_t mode = HIP_BEHAVIOR_NORMAL;

    hip_behavior_request_jump();
    hip_behavior_tick(0U, MOTION_MODE_BALANCING, &left, &right, &target, &mode);
    hip_behavior_set_imu_accel(HIP_LIFTOFF_ACCEL_G - 0.1f, true);
    hip_behavior_tick(HIP_BEHAVIOR_CROUCH_MS + 20U, MOTION_MODE_BALANCING,
                      &left, &right, &target, &mode);
    hip_behavior_tick(HIP_BEHAVIOR_CROUCH_MS + 30U, MOTION_MODE_BALANCING,
                      &left, &right, &target, &mode);
    CHECK(mode == HIP_BEHAVIOR_FLIGHT);

    hip_behavior_set_imu_accel(HIP_LANDING_ACCEL_G + 0.2f, true);
    hip_behavior_tick(HIP_BEHAVIOR_CROUCH_MS + HIP_BEHAVIOR_IMPULSE_MS + 30U,
                      MOTION_MODE_BALANCING, &left, &right, &target, &mode);
    hip_behavior_tick(HIP_BEHAVIOR_CROUCH_MS + HIP_BEHAVIOR_IMPULSE_MS + 40U,
                      MOTION_MODE_BALANCING, &left, &right, &target, &mode);
    CHECK(mode == HIP_BEHAVIOR_LANDING);
}

TEST_CASE("hip_behavior phase progress updates", "[hip][behavior]") {
    hip_behavior_init(0.6f);
    hip_state_t left = make_state(0.6f);
    hip_state_t right = make_state(0.6f);
    hip_target_t target = {};
    hip_behavior_mode_t mode = HIP_BEHAVIOR_NORMAL;

    hip_behavior_request_jump();
    hip_behavior_tick(0U, MOTION_MODE_BALANCING, &left, &right, &target, &mode);
    hip_behavior_tick(50U, MOTION_MODE_BALANCING, &left, &right, &target, &mode);
    uint8_t pct = hip_behavior_get_phase_progress();
    CHECK(pct > 0U);
    CHECK(pct < 100U);
}
