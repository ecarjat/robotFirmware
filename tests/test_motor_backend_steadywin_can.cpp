#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>

extern "C" {
#include "motor_link.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"

bool motor_backend_steadywin_can_init(void);
void motor_backend_steadywin_can_poll(void);
bool motor_backend_steadywin_can_enable(bool on);
void motor_backend_steadywin_can_set_wheel_torques(float left_Nm,
                                                    float right_Nm,
                                                    float max_Nm);
bool motor_backend_steadywin_can_get_wheel_velocities(float *left_rad_s,
                                                       float *right_rad_s);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs);

extern uint32_t g_hal_tick;
}

namespace {

constexpr uint8_t kLeftNode = 1U;
constexpr uint8_t kRightNode = 2U;
constexpr uint8_t kCmdAxisState = 0x07U;
constexpr uint8_t kCmdEncoderEstimates = 0x09U;
constexpr uint8_t kCmdControllerMode = 0x0BU;
constexpr uint8_t kCmdInputTorque = 0x0EU;
constexpr uint8_t kCmdClearErrors = 0x18U;
constexpr float kPi = 3.14159265358979323846f;

uint32_t can_id(uint8_t node_id, uint8_t command)
{
    return (static_cast<uint32_t>(node_id) << 5U) | command;
}

uint32_t read_u32(const uint8_t *data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8U) |
           (static_cast<uint32_t>(data[2]) << 16U) |
           (static_cast<uint32_t>(data[3]) << 24U);
}

float read_f32(const uint8_t *data)
{
    float value = 0.0f;
    std::memcpy(&value, data, sizeof(value));
    return value;
}

void write_f32(uint8_t *data, float value)
{
    std::memcpy(data, &value, sizeof(value));
}

void init_backend()
{
    hal_fake_fdcan_reset();
    g_hal_tick = 0U;
    REQUIRE(motor_backend_steadywin_can_init());
}

} // namespace

TEST_CASE("GDS68 wheel enable sends CAN Simple torque-mode sequence")
{
    init_backend();

    REQUIRE(motor_backend_steadywin_can_enable(true));
    REQUIRE(g_fdcan_tx_count == 8U);

    REQUIRE(g_fdcan_tx_headers[0].Identifier == can_id(kLeftNode, kCmdClearErrors));
    REQUIRE(g_fdcan_tx_headers[0].DataLength == FDCAN_DLC_BYTES_0);
    REQUIRE(g_fdcan_tx_headers[1].Identifier == can_id(kLeftNode, kCmdControllerMode));
    REQUIRE(g_fdcan_tx_headers[1].DataLength == FDCAN_DLC_BYTES_8);
    REQUIRE(read_u32(&g_fdcan_tx_data[1][0]) == 1U);
    REQUIRE(read_u32(&g_fdcan_tx_data[1][4]) == 1U);
    REQUIRE(g_fdcan_tx_headers[2].Identifier == can_id(kLeftNode, kCmdInputTorque));
    REQUIRE(read_f32(g_fdcan_tx_data[2]) == 0.0f);
    REQUIRE(g_fdcan_tx_headers[3].Identifier == can_id(kLeftNode, kCmdAxisState));
    REQUIRE(read_u32(g_fdcan_tx_data[3]) == 8U);

    REQUIRE(g_fdcan_tx_headers[4].Identifier == can_id(kRightNode, kCmdClearErrors));
    REQUIRE(g_fdcan_tx_headers[5].Identifier == can_id(kRightNode, kCmdControllerMode));
    REQUIRE(g_fdcan_tx_headers[6].Identifier == can_id(kRightNode, kCmdInputTorque));
    REQUIRE(g_fdcan_tx_headers[7].Identifier == can_id(kRightNode, kCmdAxisState));
}

TEST_CASE("GDS68 wheel torque commands use CAN Simple float32 Nm payloads")
{
    init_backend();
    REQUIRE(motor_backend_steadywin_can_enable(true));
    hal_fake_fdcan_reset();
    g_hal_tick = 10U;

    motor_backend_steadywin_can_set_wheel_torques(3.0f, -3.0f, 2.0f);

    REQUIRE(g_fdcan_tx_count == 2U);
    REQUIRE(g_fdcan_tx_headers[0].Identifier == can_id(kLeftNode, kCmdInputTorque));
    REQUIRE(read_f32(g_fdcan_tx_data[0]) == 2.0f);
    REQUIRE(g_fdcan_tx_headers[1].Identifier == can_id(kRightNode, kCmdInputTorque));
    REQUIRE(read_f32(g_fdcan_tx_data[1]) == -2.0f);
}

TEST_CASE("GDS68 periodic encoder estimates update wheel velocities")
{
    init_backend();
    g_hal_tick = 100U;

    uint8_t left_data[8] = {0U};
    uint8_t right_data[8] = {0U};
    write_f32(&left_data[0], 0.0f);
    write_f32(&left_data[4], 2.0f);
    write_f32(&right_data[0], 0.0f);
    write_f32(&right_data[4], -1.5f);
    hal_fake_fdcan_push_rx(can_id(kLeftNode, kCmdEncoderEstimates),
                           FDCAN_DLC_BYTES_8, left_data);
    hal_fake_fdcan_push_rx(can_id(kRightNode, kCmdEncoderEstimates),
                           FDCAN_DLC_BYTES_8, right_data);

    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    motor_backend_steadywin_can_poll();

    float left_rad_s = 0.0f;
    float right_rad_s = 0.0f;
    REQUIRE(motor_backend_steadywin_can_get_wheel_velocities(&left_rad_s,
                                                              &right_rad_s));
    REQUIRE(left_rad_s == Catch::Approx(4.0f * kPi));
    REQUIRE(right_rad_s == Catch::Approx(-3.0f * kPi));
}
