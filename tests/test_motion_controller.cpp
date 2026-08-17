#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <cstring>

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
    CHECK(u == Catch::Approx(-1.2f).margin(0.001f));
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

TEST_CASE("MotionController cruise mode suppresses K0 position action", "[motion_controller]")
{
    RobotParams params;
    MotionController controller(params);

    lqr_params_t lqr{};
    lqr.K[0] = 2.0f;
    lqr.K[1] = 0.0f;
    lqr.K[2] = 0.0f;
    lqr.K[3] = 0.0f;
    lqr.u_limit = 100.0f;
    lqr.theta_ref_limit = 1.0f;
    lqr.v_ref_limit = 10.0f;
    controller.setLqrParams(lqr);

    LqrSpeedSchedule sched{};
    sched.enabled = true;
    sched.cruise_enter_cmd_mps = 0.1f;
    sched.cruise_exit_cmd_mps = 0.05f;
    sched.cruise_enter_meas_mps = 10.0f;
    sched.cruise_exit_meas_mps = 10.0f;
    sched.cruise_blend_tau_s = 0.0f;
    controller.setLqrSpeedSchedule(sched);

    controller.setRequestedMode(InnerLongMode::LQR);
    controller.resetPidState();
    controller.setTargetVelocity(1.0f);

    StateEstimate state{};
    state.x = 5.0f;
    state.xDot = 0.0f;
    state.theta = 0.0f;
    state.thetaDot = 0.0f;

    (void)controller.computeControl(state, 0.01f);
    InnerCtrlDiag diag{};
    REQUIRE(controller.getInnerCtrlDiag(diag));
    CHECK(diag.cruise_mode);
    CHECK(diag.cruise_alpha == Catch::Approx(1.0f));
    CHECK(diag.k0_eff == Catch::Approx(0.0f).margin(1e-6f));
}

TEST_CASE("MotionController cruise hysteresis uses command and measured speed", "[motion_controller]")
{
    RobotParams params;
    MotionController controller(params);

    lqr_params_t lqr{};
    lqr.K[0] = 0.0f;
    lqr.K[1] = 0.0f;
    lqr.K[2] = 0.0f;
    lqr.K[3] = 0.0f;
    lqr.u_limit = 100.0f;
    lqr.theta_ref_limit = 1.0f;
    lqr.v_ref_limit = 10.0f;
    controller.setLqrParams(lqr);

    LqrSpeedSchedule sched{};
    sched.enabled = true;
    sched.cruise_enter_cmd_mps = 0.6f;
    sched.cruise_exit_cmd_mps = 0.25f;
    sched.cruise_enter_meas_mps = 0.5f;
    sched.cruise_exit_meas_mps = 0.2f;
    sched.cruise_blend_tau_s = 0.0f;
    controller.setLqrSpeedSchedule(sched);

    controller.setRequestedMode(InnerLongMode::LQR);
    controller.resetPidState();

    StateEstimate state{};
    state.theta = 0.0f;
    state.thetaDot = 0.0f;

    controller.setTargetVelocity(0.7f);
    state.xDot = 0.0f;
    (void)controller.computeControl(state, 0.01f);
    InnerCtrlDiag diag{};
    REQUIRE(controller.getInnerCtrlDiag(diag));
    CHECK(diag.cruise_mode);

    controller.setTargetVelocity(0.1f);
    state.xDot = 0.3f;  // still above measured exit threshold
    (void)controller.computeControl(state, 0.01f);
    REQUIRE(controller.getInnerCtrlDiag(diag));
    CHECK(diag.cruise_mode);

    state.xDot = 0.1f;  // below both exit thresholds
    (void)controller.computeControl(state, 0.01f);
    REQUIRE(controller.getInnerCtrlDiag(diag));
    CHECK_FALSE(diag.cruise_mode);
}

TEST_CASE("MotionController cruise scheduling expands effective v_ref_limit", "[motion_controller]")
{
    RobotParams params;
    MotionController controller(params);

    lqr_params_t lqr{};
    lqr.K[0] = 1.0f;
    lqr.K[1] = 0.0f;
    lqr.K[2] = 0.0f;
    lqr.K[3] = 0.0f;
    lqr.u_limit = 100.0f;
    lqr.theta_ref_limit = 1.0f;
    lqr.v_ref_limit = 0.2f;
    controller.setLqrParams(lqr);

    LqrSpeedSchedule sched{};
    sched.enabled = true;
    sched.cruise_enter_cmd_mps = 0.1f;
    sched.cruise_exit_cmd_mps = 0.05f;
    sched.cruise_enter_meas_mps = 10.0f;
    sched.cruise_exit_meas_mps = 10.0f;
    sched.cruise_blend_tau_s = 0.0f;
    sched.v_ref_margin_mps = 0.3f;
    controller.setLqrSpeedSchedule(sched);

    controller.setRequestedMode(InnerLongMode::LQR);
    controller.resetPidState();
    controller.setTargetVelocity(1.5f);

    StateEstimate state{};
    state.x = 10.0f;
    state.xDot = 0.0f;
    state.theta = 0.0f;
    state.thetaDot = 0.0f;

    (void)controller.computeControl(state, 0.01f);
    InnerCtrlDiag diag{};
    REQUIRE(controller.getInnerCtrlDiag(diag));
    CHECK(diag.v_ref_limit_eff > 0.2f);
    const float expected_vref = params.maxForwardVelocity + sched.v_ref_margin_mps;
    CHECK(diag.v_ref_limit_eff == Catch::Approx(expected_vref).margin(0.05f));
}

TEST_CASE("MotionController cruise scheduling boosts yaw damping", "[motion_controller]")
{
    RobotParams params;
    MotionController controller(params);

    balance_gains_t gains{};
    std::memset(&gains, 0, sizeof(gains));
    gains.K_yawDamp = 1.0f;
    gains.K_turn = 0.0f;
    controller.setBalanceGains(gains);

    lqr_params_t lqr{};
    lqr.K[0] = 0.0f;
    lqr.K[1] = 0.0f;
    lqr.K[2] = 0.0f;
    lqr.K[3] = 0.0f;
    lqr.u_limit = 100.0f;
    lqr.theta_ref_limit = 1.0f;
    lqr.v_ref_limit = 10.0f;
    controller.setLqrParams(lqr);

    LqrSpeedSchedule sched{};
    sched.enabled = true;
    sched.cruise_enter_cmd_mps = 0.1f;
    sched.cruise_exit_cmd_mps = 0.05f;
    sched.cruise_enter_meas_mps = 10.0f;
    sched.cruise_exit_meas_mps = 10.0f;
    sched.cruise_blend_tau_s = 0.0f;
    sched.yaw_damp_cruise_mult = 2.0f;
    controller.setLqrSpeedSchedule(sched);

    controller.setRequestedMode(InnerLongMode::LQR);
    controller.resetPidState();

    StateEstimate state{};
    state.theta = 0.0f;
    state.thetaDot = 0.0f;
    state.x = 0.0f;
    state.xDot = 0.0f;

    controller.setTargetVelocity(0.0f);
    controller.setYawRates(1.0f, 1.0f);
    (void)controller.computeControl(state, 0.01f);
    InnerCtrlDiag rest_diag{};
    REQUIRE(controller.getInnerCtrlDiag(rest_diag));
    CHECK(rest_diag.yaw_damp_eff == Catch::Approx(1.0f).margin(1e-6f));
    CHECK(rest_diag.u_diff_cmd == Catch::Approx(-1.0f).margin(1e-4f));

    controller.setTargetVelocity(1.0f);
    (void)controller.computeControl(state, 0.01f);
    InnerCtrlDiag cruise_diag{};
    REQUIRE(controller.getInnerCtrlDiag(cruise_diag));
    CHECK(cruise_diag.cruise_mode);
    CHECK(cruise_diag.yaw_damp_eff == Catch::Approx(2.0f).margin(1e-6f));
    CHECK(cruise_diag.u_diff_cmd == Catch::Approx(-2.0f).margin(1e-4f));
}

TEST_CASE("MotionController stop mode smooths release and limits position pullback", "[motion_controller]")
{
    RobotParams params;
    MotionController controller(params);

    lqr_params_t lqr{};
    lqr.K[0] = 2.0f;
    lqr.K[1] = 1.0f;
    lqr.K[2] = 0.0f;
    lqr.K[3] = 0.0f;
    lqr.u_limit = 100.0f;
    lqr.theta_ref_limit = 1.0f;
    lqr.v_ref_limit = 10.0f;
    controller.setLqrParams(lqr);

    LqrSpeedSchedule sched{};
    sched.enabled = false;  // isolate stop-mode behavior from cruise scheduling
    controller.setLqrSpeedSchedule(sched);

    controller.setRequestedMode(InnerLongMode::LQR);
    controller.resetPidState();

    StateEstimate state{};
    state.theta = 0.0f;
    state.thetaDot = 0.0f;
    state.xDot = 1.0f;
    state.x = 0.0f;

    controller.setTargetVelocity(1.0f);
    (void)controller.computeControl(state, 0.1f);  // prime references

    controller.setTargetVelocity(0.0f);           // abrupt release
    state.x = 0.1f;
    (void)controller.computeControl(state, 0.1f); // stop mode engages
    InnerCtrlDiag diag{};
    REQUIRE(controller.getInnerCtrlDiag(diag));
    CHECK(diag.k0_eff == Catch::Approx(0.0f).margin(1e-6f));
    CHECK(diag.u_sum_lqr == Catch::Approx(0.0f).margin(0.02f));
    CHECK(diag.x_err == Catch::Approx(0.0f).margin(0.02f));

    state.x = 0.2f;
    (void)controller.computeControl(state, 0.1f); // deceleration ramp toward zero
    REQUIRE(controller.getInnerCtrlDiag(diag));
    CHECK(diag.k0_eff == Catch::Approx(0.0f).margin(1e-6f));
    CHECK(diag.u_sum_lqr > -0.2f);
    CHECK(diag.x_err == Catch::Approx(0.0f).margin(0.03f));
}

TEST_CASE("MotionController stop mode exits near rest and re-anchors xRef", "[motion_controller]")
{
    RobotParams params;
    MotionController controller(params);

    lqr_params_t lqr{};
    lqr.K[0] = 2.0f;
    lqr.K[1] = 0.0f;
    lqr.K[2] = 0.0f;
    lqr.K[3] = 0.0f;
    lqr.u_limit = 100.0f;
    lqr.theta_ref_limit = 1.0f;
    lqr.v_ref_limit = 10.0f;
    controller.setLqrParams(lqr);

    LqrSpeedSchedule sched{};
    sched.enabled = false;
    controller.setLqrSpeedSchedule(sched);

    controller.setRequestedMode(InnerLongMode::LQR);
    controller.resetPidState();

    StateEstimate state{};
    state.theta = 0.0f;
    state.thetaDot = 0.0f;
    state.xDot = 1.0f;
    state.x = 0.0f;

    controller.setTargetVelocity(1.0f);
    (void)controller.computeControl(state, 0.1f);

    controller.setTargetVelocity(0.0f);
    state.x = 0.1f;
    (void)controller.computeControl(state, 0.1f);  // enter stop mode

    state.xDot = 0.0f;
    for (int i = 0; i < 20; ++i) {
        (void)controller.computeControl(state, 0.1f);
    }

    InnerCtrlDiag diag{};
    REQUIRE(controller.getInnerCtrlDiag(diag));
    CHECK_FALSE(diag.cruise_mode);
    CHECK(diag.k0_eff == Catch::Approx(2.0f).margin(1e-6f));
    CHECK(diag.x_err == Catch::Approx(0.0f).margin(0.02f));
}

TEST_CASE("MotionController stop mode engages on ramped release", "[motion_controller]")
{
    RobotParams params;
    MotionController controller(params);

    lqr_params_t lqr{};
    lqr.K[0] = 2.0f;
    lqr.K[1] = 0.0f;
    lqr.K[2] = 0.0f;
    lqr.K[3] = 0.0f;
    lqr.u_limit = 100.0f;
    lqr.theta_ref_limit = 1.0f;
    lqr.v_ref_limit = 10.0f;
    controller.setLqrParams(lqr);

    LqrSpeedSchedule sched{};
    sched.enabled = false;
    controller.setLqrSpeedSchedule(sched);

    controller.setRequestedMode(InnerLongMode::LQR);
    controller.resetPidState();

    StateEstimate state{};
    state.theta = 0.0f;
    state.thetaDot = 0.0f;
    state.xDot = 1.0f;
    state.x = 0.0f;

    controller.setTargetVelocity(1.0f);
    (void)controller.computeControl(state, 0.1f);

    const float ramp_cmds[] = {0.8f, 0.6f, 0.4f, 0.2f, 0.1f, 0.04f, 0.0f};
    InnerCtrlDiag diag{};
    for (float v : ramp_cmds) {
        controller.setTargetVelocity(v);
        state.x += state.xDot * 0.1f;
        (void)controller.computeControl(state, 0.1f);
    }
    REQUIRE(controller.getInnerCtrlDiag(diag));
    CHECK(diag.k0_eff == Catch::Approx(0.0f).margin(1e-6f));
}

TEST_CASE("MotionController suppresses K0 in in-place turn mode", "[motion_controller]")
{
    RobotParams params;
    MotionController controller(params);

    lqr_params_t lqr{};
    lqr.K[0] = 2.0f;
    lqr.K[1] = 0.0f;
    lqr.K[2] = 0.0f;
    lqr.K[3] = 0.0f;
    lqr.u_limit = 100.0f;
    lqr.theta_ref_limit = 1.0f;
    lqr.v_ref_limit = 10.0f;
    controller.setLqrParams(lqr);

    LqrSpeedSchedule sched{};
    sched.enabled = false;
    controller.setLqrSpeedSchedule(sched);

    controller.setRequestedMode(InnerLongMode::LQR);
    controller.resetPidState();

    StateEstimate state{};
    state.theta = 0.0f;
    state.thetaDot = 0.0f;
    state.xDot = 0.0f;
    state.x = 0.0f;

    controller.setTeleopCommands(0.0f, 0.0f);
    (void)controller.computeControl(state, 0.01f);  // initialize xRef

    state.x = 0.5f;  // create position error that K0 would normally correct
    controller.setTeleopCommands(0.0f, 1.0f);
    (void)controller.computeControl(state, 0.01f);

    InnerCtrlDiag diag{};
    REQUIRE(controller.getInnerCtrlDiag(diag));
    CHECK(diag.k0_eff == Catch::Approx(0.0f).margin(1e-6f));
}

TEST_CASE("MotionController yaw governor reserves balance headroom", "[motion_controller]")
{
    RobotParams params;
    MotionController controller(params);

    balance_gains_t gains{};
    std::memset(&gains, 0, sizeof(gains));
    gains.K_turn = 20.0f;     // large turn request
    gains.K_yawDamp = 0.0f;   // isolate command path
    controller.setBalanceGains(gains);

    lqr_params_t lqr{};
    lqr.K[0] = 0.0f;
    lqr.K[1] = 0.0f;
    lqr.K[2] = 100.0f;  // drive u_sum near limit
    lqr.K[3] = 0.0f;
    lqr.u_limit = 34.0f;
    lqr.theta_ref_limit = 1.0f;
    lqr.v_ref_limit = 10.0f;
    controller.setLqrParams(lqr);

    LqrSpeedSchedule sched{};
    sched.enabled = false;
    controller.setLqrSpeedSchedule(sched);

    controller.setRequestedMode(InnerLongMode::LQR);
    controller.resetPidState();

    StateEstimate state{};
    state.theta = 0.33f;  // u_sum ~= -33 Nm
    state.thetaDot = 0.0f;
    state.xDot = 0.0f;
    state.x = 0.0f;

    controller.setTeleopCommands(0.0f, 1.0f);
    (void)controller.computeControl(state, 0.01f);

    InnerCtrlDiag diag{};
    REQUIRE(controller.getInnerCtrlDiag(diag));
    CHECK(std::fabs(diag.u_sum_cmd) > 30.0f);
    CHECK(diag.u_diff_cmd == Catch::Approx(0.0f).margin(1e-3f));
}
