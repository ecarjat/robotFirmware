#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "lqr_lut.h"
#include "lqr_lut_data.h"

#include <cmath>

TEST_CASE("lqr_lut_data is finite and monotonic", "[lqr][lut]")
{
    for (size_t idx = 0; idx < LQR_LUT_SIZE; ++idx) {
        REQUIRE(std::isfinite(kHipLut[idx]));
        if (idx > 0) {
            CHECK(kHipLut[idx] > kHipLut[idx - 1]);
        }

        for (size_t gain = 0; gain < 4; ++gain) {
            CHECK(std::isfinite(kLqrLut[idx][gain]));
        }
        CHECK(std::isfinite(kThetaEq[idx]));
        CHECK(std::isfinite(kUEq[idx]));
    }
}

TEST_CASE("lqr_lut_eval clamps to endpoints", "[lqr][lut]")
{
    float K[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    const float below_range = kHipLut[0] - 1.0f;
    const float above_range = kHipLut[LQR_LUT_SIZE - 1] + 1.0f;

    REQUIRE(lqr_lut_eval(below_range, K));
    for (size_t gain = 0; gain < 4; ++gain) {
        CHECK(K[gain] == Catch::Approx(kLqrLut[0][gain]));
    }

    REQUIRE(lqr_lut_eval(above_range, K));
    for (size_t gain = 0; gain < 4; ++gain) {
        CHECK(K[gain] == Catch::Approx(kLqrLut[LQR_LUT_SIZE - 1][gain]));
    }
}

TEST_CASE("lqr_lut_eval interpolates between adjacent rows", "[lqr][lut]")
{
    float K[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    constexpr float t = 0.37f;

    for (size_t idx = 0; idx + 1 < LQR_LUT_SIZE; ++idx) {
        const float x0 = kHipLut[idx];
        const float x1 = kHipLut[idx + 1];
        const float x = x0 + (x1 - x0) * t;

        REQUIRE(lqr_lut_eval(x, K));
        for (size_t gain = 0; gain < 4; ++gain) {
            const float expected =
                kLqrLut[idx][gain] + (kLqrLut[idx + 1][gain] - kLqrLut[idx][gain]) * t;
            CHECK(K[gain] == Catch::Approx(expected));
        }
    }
}

TEST_CASE("lqr_lut_eval_full interpolates equilibrium terms", "[lqr][lut]")
{
    constexpr float t = 0.61f;

    for (size_t idx = 0; idx + 1 < LQR_LUT_SIZE; ++idx) {
        const float x0 = kHipLut[idx];
        const float x1 = kHipLut[idx + 1];
        const float x = x0 + (x1 - x0) * t;

        float K[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        float theta_eq = 0.0f;
        float u_eq = 0.0f;
        REQUIRE(lqr_lut_eval_full(x, K, &theta_eq, &u_eq));

        const float expected_theta =
            kThetaEq[idx] + (kThetaEq[idx + 1] - kThetaEq[idx]) * t;
        const float expected_u =
            kUEq[idx] + (kUEq[idx + 1] - kUEq[idx]) * t;

        CHECK(theta_eq == Catch::Approx(expected_theta));
        CHECK(u_eq == Catch::Approx(expected_u));
    }
}
