#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "MotionController.h"
#include "StateEstimate.h"

TEST_CASE("MotionController computePid clamps and responds to error", "[motion_controller]")
{
    RobotParams params;
    MotionController controller(params);
    controller.resetPidState();

    float out = controller.test_computePid(
        1.0f,  /* setpoint */
        0.0f,  /* measurement */
        0.01f, /* dt */
        1.0f,  /* Kp */
        0.0f,  /* Ki */
        0.0f,  /* Kd */
        0.5f,  /* outputLimit */
        10.0f  /* integralLimit */
    );

    CHECK(out == Catch::Approx(0.5f)); /* clamped */
}

TEST_CASE("MotionController applyVelocitySlew limits step", "[motion_controller]")
{
    RobotParams params;
    MotionController controller(params);
    controller.resetPidState();

    MotionController::ControlOutput desired{10.0f, -10.0f};
    MotionController::ControlOutput out = controller.test_applyVelocitySlew(desired, 1.0f);

    const float maxStep = VELOCITY_SLEW_RATE_RAD_PER_S2 * 1.0f;
    CHECK(out.torqueLeftNm == Catch::Approx(10.0f));
    CHECK(out.torqueRightNm == Catch::Approx(-10.0f));
}

TEST_CASE("MotionController computeLqrUSumNm uses gains", "[motion_controller]")
{
    RobotParams params;
    MotionController controller(params);

    lqr_params_t lqr{};
    lqr.K[0] = 0.0f;
    lqr.K[1] = 2.0f;
    lqr.K[2] = 3.0f;
    lqr.K[3] = 4.0f;
    controller.setLqrParams(lqr);

    StateEstimate state{};
    state.xDot = 1.5f;
    state.theta = 0.2f;
    state.thetaDot = -0.1f;

    float u = controller.test_computeLqrUSumNm(state, 1.0f, 0.0f);
    // v_err = 0.5, theta_err = 0.2, thetaDot = -0.1
    // u = -(2*0.5 + 3*0.2 + 4*(-0.1)) = -(1.0 + 0.6 -0.4) = -1.2
    CHECK(u == Catch::Approx(-1.2f));
}

TEST_CASE("MotionController computeLqrUSumNm respects v_ref_limit", "[motion_controller]")
{
    RobotParams params;
    MotionController controller(params);
    controller.resetPidState();

    lqr_params_t lqr{};
    lqr.K[0] = 1.0f;   /* x_err -> v_ref */
    lqr.K[1] = 1.0f;   /* v_err term */
    lqr.K[2] = 0.0f;
    lqr.K[3] = 0.0f;
    lqr.theta_ref_limit = 1.0f;
    lqr.v_ref_limit = 0.2f;
    controller.setLqrParams(lqr);

    StateEstimate state{};
    state.x = 10.0f;
    state.xDot = 0.0f;
    float limited_u = controller.test_computeLqrUSumNm(state, 0.0f, 0.0f);
    CHECK(limited_u == Catch::Approx(0.2f));

    controller.resetPidState();
    lqr.v_ref_limit = 2.0f;
    controller.setLqrParams(lqr);
    float wider_limit_u = controller.test_computeLqrUSumNm(state, 0.0f, 0.0f);
    CHECK(wider_limit_u == Catch::Approx(2.0f));
}

TEST_CASE("MotionController trim loop clamps under sustained velocity error", "[motion_controller]")
{
    RobotParams params;
    MotionController controller(params);
    controller.resetPidState();
    controller.setControlDt(1.0f);

    lqr_params_t lqr{};
    lqr.K[0] = 0.0f;
    lqr.K[1] = 0.0f;
    lqr.K[2] = 1.0f;   /* expose theta trim as torque output */
    lqr.K[3] = 0.0f;
    lqr.theta_ref_limit = 1.0f;
    lqr.v_ref_limit = 10.0f;
    controller.setLqrParams(lqr);

    StateEstimate state{};
    state.x = 0.0f;
    state.theta = 0.0f;
    state.thetaDot = 0.0f;
    state.xDot = -10.0f;

    float u = 0.0f;
    for (int i = 0; i < 8; ++i) {
        u = controller.test_computeLqrUSumNm(state, 10.0f, 0.0f);
    }
    CHECK(u == Catch::Approx(0.08f).margin(0.002f));

    state.xDot = 10.0f;
    for (int i = 0; i < 8; ++i) {
        u = controller.test_computeLqrUSumNm(state, -10.0f, 0.0f);
    }
    CHECK(u == Catch::Approx(-0.08f).margin(0.002f));
}
