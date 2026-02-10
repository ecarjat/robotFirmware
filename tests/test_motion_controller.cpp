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
