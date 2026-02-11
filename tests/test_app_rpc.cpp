#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cstring>

extern "C" {
#include "app_rpc.h"
#include "app_main.h"
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
