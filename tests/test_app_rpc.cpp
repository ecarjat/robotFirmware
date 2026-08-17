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
