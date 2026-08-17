#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cstddef>
#include <cstring>
#include <limits>

extern "C" {
#include "app_rpc.h"
#include "app_main.h"
#include "app_motor.h"
#include "motion_control.h"
#include "param_storage.h"
#include "robot_protocol.h"
#include "hip_control.h"
}

extern "C" {
extern uint8_t g_app_link_last_type;
extern uint16_t g_app_link_last_len;
extern uint8_t g_app_link_last_payload[ROBOT_FRAME_MAX_PAYLOAD];
extern bool g_param_can_save;
extern int g_param_save_calls;
extern int g_param_save_rc;
void hip_control_fake_set_states(const hip_state_t *left, const hip_state_t *right);
void hip_control_fake_set_program_result(bool result);
void hip_control_fake_get_program(uint8_t *current_node_id, uint8_t *new_node_id, bool *save);
}

static robot_frame_t make_rpc_frame(uint8_t method, uint16_t flags)
{
    robot_frame_t frame = {};
    frame.hdr.type = ROBOT_MSG_RPC_REQ;
    frame.hdr.flags = flags;
    frame.hdr.len = sizeof(robot_rpc_param_t);
    robot_rpc_param_t req = {};
    req.method = method;
    req.flags = flags;
    req.offset = 0U;
    req.length = 0U;
    memcpy(frame.payload, &req, sizeof(req));
    return frame;
}

static robot_frame_t make_rpc_frame_with_payload(uint8_t method, uint16_t flags,
                                                 const void *data, uint16_t data_len)
{
    robot_frame_t frame = {};
    frame.hdr.type = ROBOT_MSG_RPC_REQ;
    frame.hdr.flags = flags;
    frame.hdr.len = sizeof(robot_rpc_param_t) + data_len;
    robot_rpc_param_t req = {};
    req.method = method;
    req.flags = flags;
    req.offset = 0U;
    req.length = data_len;
    memcpy(frame.payload, &req, sizeof(req));
    if (data_len > 0U && data != nullptr) {
        memcpy(frame.payload + sizeof(req), data, data_len);
    }
    return frame;
}

static robot_frame_t make_set_param_frame(size_t offset, const void *data,
                                          uint16_t data_len)
{
    robot_frame_t frame = make_rpc_frame_with_payload(ROBOT_RPC_METHOD_SET_PARAM,
                                                       0U, data, data_len);
    auto *req = reinterpret_cast<robot_rpc_param_t *>(frame.payload);
    req->offset = static_cast<uint16_t>(offset);
    req->length = data_len;
    return frame;
}

static void set_identity(float matrix[9])
{
    std::memset(matrix, 0, sizeof(float) * 9U);
    matrix[0] = 1.0f;
    matrix[4] = 1.0f;
    matrix[8] = 1.0f;
}

static void set_valid_params()
{
    g_robot_params = {};
    g_robot_params.motor_direction[0] = 1;
    g_robot_params.motor_direction[1] = 1;
    g_robot_params.wheel_radius_m = 0.033f;
    g_robot_params.wheel_base_m = 0.15f;
    set_identity(g_robot_params.imu_bmi270.rotation);
    set_identity(g_robot_params.imu_icm42688.rotation);
    set_identity(g_robot_params.mag_bmm150.soft_iron);

    g_robot_params.balance.max_tilt_ref = 0.15f;
    g_robot_params.balance.alpha_yaw = 0.8f;
    g_robot_params.balance.IqMax = 3.0f;
    g_robot_params.balance.thetaKill = 0.785f;
    g_robot_params.balance.iV_max = 0.5f;

    g_robot_params.lqr.u_limit = 34.0f;
    g_robot_params.lqr.du_limit = 100.0f;
    g_robot_params.lqr.theta_ref_limit = 0.01f;
    g_robot_params.lqr.v_ref_limit = 2.0f;
    g_robot_params.lqr.default_mode = 0U;

    g_robot_params.max_linear_vel_mps = 0.5f;
    g_robot_params.max_angular_vel_rps = 2.0f;
    g_robot_params.max_linear_accel_mps2 = 1.0f;
    g_robot_params.max_angular_accel_rps2 = 4.0f;
    g_robot_params.control_rate_hz = 400.0f;
    g_robot_params.uart_baudrate = 115200U;
    g_robot_params.robot_id = 1U;
    g_robot_params.adc_voltage_multiplier = 1.0f;
    g_robot_params.hip_left_dir_sign = 1;
    g_robot_params.hip_right_dir_sign = 1;
    s_motor_manual = {};
    motion_control_set_mode(MOTION_MODE_DISARMED);
}

TEST_CASE("app_rpc hip calib returns NOT_READY when hip invalid", "[rpc][hip]") {
    hip_state_t left = {};
    hip_state_t right = {};
    left.valid = 0U;
    right.valid = 0U;
    hip_control_fake_set_states(&left, &right);

    robot_frame_t frame = make_rpc_frame(ROBOT_RPC_METHOD_HIP_CALIB_ZERO, 0U);
    app_rpc_handle(&frame);

    REQUIRE(g_app_link_last_type == ROBOT_MSG_RPC_RESP);
    auto *resp = reinterpret_cast<const robot_rpc_param_t *>(g_app_link_last_payload);
    CHECK(resp->flags == ROBOT_RPC_STATUS_NOT_READY);
}

TEST_CASE("app_rpc hip calib updates params and saves", "[rpc][hip]") {
    hip_state_t left = {};
    hip_state_t right = {};
    left.valid = 1U;
    right.valid = 1U;
    left.pos_rev = 0.25f;
    right.pos_rev = -0.5f;
    hip_control_fake_set_states(&left, &right);

    g_param_can_save = true;
    g_param_save_calls = 0;
    g_param_save_rc = PARAM_OK;

    robot_frame_t frame = make_rpc_frame(ROBOT_RPC_METHOD_HIP_CALIB_ZERO, ROBOT_RPC_FLAG_SAVE);
    app_rpc_handle(&frame);

    REQUIRE(g_app_link_last_type == ROBOT_MSG_RPC_RESP);
    auto *resp = reinterpret_cast<const robot_rpc_param_t *>(g_app_link_last_payload);
    CHECK(resp->flags == ROBOT_RPC_STATUS_OK);
    CHECK(g_robot_params.hip_left_zero_offset_rev == Catch::Approx(0.25f));
    CHECK(g_robot_params.hip_right_zero_offset_rev == Catch::Approx(-0.5f));
    CHECK(g_param_save_calls == 1);
}

TEST_CASE("app_rpc hip calib save returns NOT_READY when storage not available", "[rpc][hip]") {
    hip_state_t left = {};
    hip_state_t right = {};
    left.valid = 1U;
    right.valid = 1U;
    left.pos_rev = 0.3f;
    right.pos_rev = -0.2f;
    hip_control_fake_set_states(&left, &right);

    g_param_can_save = false;
    g_param_save_calls = 0;
    g_param_save_rc = PARAM_OK;

    robot_frame_t frame = make_rpc_frame(ROBOT_RPC_METHOD_HIP_CALIB_ZERO, ROBOT_RPC_FLAG_SAVE);
    app_rpc_handle(&frame);

    REQUIRE(g_app_link_last_type == ROBOT_MSG_RPC_RESP);
    auto *resp = reinterpret_cast<const robot_rpc_param_t *>(g_app_link_last_payload);
    CHECK(resp->flags == ROBOT_RPC_STATUS_NOT_READY);
    CHECK(g_param_save_calls == 0);
}

TEST_CASE("app_rpc hip calib save returns STORAGE on save failure", "[rpc][hip]") {
    hip_state_t left = {};
    hip_state_t right = {};
    left.valid = 1U;
    right.valid = 1U;
    left.pos_rev = -0.125f;
    right.pos_rev = 0.875f;
    hip_control_fake_set_states(&left, &right);

    g_param_can_save = true;
    g_param_save_calls = 0;
    g_param_save_rc = PARAM_ERR_FLASH;

    robot_frame_t frame = make_rpc_frame(ROBOT_RPC_METHOD_HIP_CALIB_ZERO, ROBOT_RPC_FLAG_SAVE);
    app_rpc_handle(&frame);

    REQUIRE(g_app_link_last_type == ROBOT_MSG_RPC_RESP);
    auto *resp = reinterpret_cast<const robot_rpc_param_t *>(g_app_link_last_payload);
    CHECK(resp->flags == ROBOT_RPC_STATUS_STORAGE);
    CHECK(g_param_save_calls == 1);
    CHECK(g_robot_params.hip_left_zero_offset_rev == Catch::Approx(-0.125f));
    CHECK(g_robot_params.hip_right_zero_offset_rev == Catch::Approx(0.875f));
}

TEST_CASE("app_rpc CAN Simple set node id forwards programming request", "[rpc][motor]") {
    robot_rpc_can_simple_node_id_t req = {};
    req.current_node_id = 0U;
    req.new_node_id = 3U;
    req.flags = ROBOT_CAN_SIMPLE_PROG_FLAG_SAVE;
    hip_control_fake_set_program_result(true);

    robot_frame_t frame =
        make_rpc_frame_with_payload(ROBOT_RPC_METHOD_CAN_SIMPLE_SET_NODE_ID, 0U,
                                    &req, sizeof(req));
    app_rpc_handle(&frame);

    REQUIRE(g_app_link_last_type == ROBOT_MSG_RPC_RESP);
    auto *resp = reinterpret_cast<const robot_rpc_param_t *>(g_app_link_last_payload);
    CHECK(resp->flags == ROBOT_RPC_STATUS_OK);
    CHECK(resp->offset == 0U);
    CHECK(resp->length == 3U);
    uint8_t current = 0xFFU;
    uint8_t next = 0xFFU;
    bool save = false;
    hip_control_fake_get_program(&current, &next, &save);
    CHECK(current == 0U);
    CHECK(next == 3U);
    CHECK(save);
}

TEST_CASE("app_rpc CAN Simple set node id rejects out of range ids", "[rpc][motor]") {
    robot_rpc_can_simple_node_id_t req = {};
    req.current_node_id = 0U;
    req.new_node_id = 64U;

    robot_frame_t frame =
        make_rpc_frame_with_payload(ROBOT_RPC_METHOD_CAN_SIMPLE_SET_NODE_ID, 0U,
                                    &req, sizeof(req));
    app_rpc_handle(&frame);

    REQUIRE(g_app_link_last_type == ROBOT_MSG_RPC_RESP);
    auto *resp = reinterpret_cast<const robot_rpc_param_t *>(g_app_link_last_payload);
    CHECK(resp->flags == ROBOT_RPC_STATUS_BAD_PARAM);
}

TEST_CASE("app_rpc SET_PARAM accepts a valid typed field update", "[rpc][params]") {
    set_valid_params();
    const float updated_rate_hz = 500.0f;
    robot_frame_t frame = make_set_param_frame(
        offsetof(robot_params_t, control_rate_hz), &updated_rate_hz,
        sizeof(updated_rate_hz));

    app_rpc_handle(&frame);

    REQUIRE(g_app_link_last_type == ROBOT_MSG_RPC_RESP);
    auto *resp = reinterpret_cast<const robot_rpc_param_t *>(g_app_link_last_payload);
    CHECK(resp->flags == ROBOT_RPC_STATUS_OK);
    CHECK(g_robot_params.control_rate_hz == Catch::Approx(updated_rate_hz));
}

TEST_CASE("app_rpc SET_PARAM rejects non-finite and out-of-range values", "[rpc][params]") {
    set_valid_params();
    const float original_rate_hz = g_robot_params.control_rate_hz;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    robot_frame_t frame = make_set_param_frame(
        offsetof(robot_params_t, control_rate_hz), &nan, sizeof(nan));

    app_rpc_handle(&frame);

    auto *resp = reinterpret_cast<const robot_rpc_param_t *>(g_app_link_last_payload);
    CHECK(resp->flags == ROBOT_RPC_STATUS_BAD_PARAM);
    CHECK(g_robot_params.control_rate_hz == Catch::Approx(original_rate_hz));

    const float infinity = std::numeric_limits<float>::infinity();
    const size_t kp_theta_offset = offsetof(robot_params_t, balance) +
                                   offsetof(balance_gains_t, Kp_theta);
    frame = make_set_param_frame(kp_theta_offset, &infinity, sizeof(infinity));
    app_rpc_handle(&frame);

    resp = reinterpret_cast<const robot_rpc_param_t *>(g_app_link_last_payload);
    CHECK(resp->flags == ROBOT_RPC_STATUS_BAD_PARAM);
    CHECK(g_robot_params.balance.Kp_theta == Catch::Approx(0.0f));

    const float excessive_rate_hz = 2001.0f;
    frame = make_set_param_frame(offsetof(robot_params_t, control_rate_hz),
                                 &excessive_rate_hz, sizeof(excessive_rate_hz));
    app_rpc_handle(&frame);

    resp = reinterpret_cast<const robot_rpc_param_t *>(g_app_link_last_payload);
    CHECK(resp->flags == ROBOT_RPC_STATUS_BAD_PARAM);
    CHECK(g_robot_params.control_rate_hz == Catch::Approx(original_rate_hz));

    const float negative_radius = -0.1f;
    frame = make_set_param_frame(offsetof(robot_params_t, wheel_radius_m),
                                 &negative_radius, sizeof(negative_radius));
    app_rpc_handle(&frame);

    resp = reinterpret_cast<const robot_rpc_param_t *>(g_app_link_last_payload);
    CHECK(resp->flags == ROBOT_RPC_STATUS_BAD_PARAM);
    CHECK(g_robot_params.wheel_radius_m == Catch::Approx(0.033f));
}

TEST_CASE("app_rpc SET_PARAM rejects malformed calibration matrices", "[rpc][params]") {
    set_valid_params();
    const float zero = 0.0f;
    const size_t rotation_offset = offsetof(robot_params_t, imu_bmi270) +
                                   offsetof(imu_calib_t, rotation);
    robot_frame_t frame = make_set_param_frame(rotation_offset, &zero,
                                               sizeof(zero));

    app_rpc_handle(&frame);

    REQUIRE(g_app_link_last_type == ROBOT_MSG_RPC_RESP);
    auto *resp = reinterpret_cast<const robot_rpc_param_t *>(g_app_link_last_payload);
    CHECK(resp->flags == ROBOT_RPC_STATUS_BAD_PARAM);
    CHECK(g_robot_params.imu_bmi270.rotation[0] == Catch::Approx(1.0f));
}

TEST_CASE("app_rpc SET_PARAM is blocked while motors can be live", "[rpc][params]") {
    set_valid_params();
    const float updated_rate_hz = 500.0f;
    robot_frame_t frame = make_set_param_frame(
        offsetof(robot_params_t, control_rate_hz), &updated_rate_hz,
        sizeof(updated_rate_hz));

    motion_control_set_mode(MOTION_MODE_BALANCING);
    app_rpc_handle(&frame);
    auto *resp = reinterpret_cast<const robot_rpc_param_t *>(g_app_link_last_payload);
    CHECK(resp->flags == ROBOT_RPC_STATUS_NOT_READY);
    CHECK(g_robot_params.control_rate_hz == Catch::Approx(400.0f));

    motion_control_set_mode(MOTION_MODE_DISARMED);
    s_motor_manual.enabled = 1U;
    app_rpc_handle(&frame);
    resp = reinterpret_cast<const robot_rpc_param_t *>(g_app_link_last_payload);
    CHECK(resp->flags == ROBOT_RPC_STATUS_NOT_READY);
    CHECK(g_robot_params.control_rate_hz == Catch::Approx(400.0f));
    s_motor_manual = {};
}
