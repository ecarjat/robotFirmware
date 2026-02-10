#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "lqr_lut.h"

TEST_CASE("lqr_lut_eval clamps to endpoints", "[lqr][lut]")
{
    float K[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    REQUIRE(lqr_lut_eval(0.1f, K));
    CHECK(K[0] == Catch::Approx(-0.000201f));
    CHECK(K[1] == Catch::Approx(-107.414622f));

    REQUIRE(lqr_lut_eval(2.0f, K));
    CHECK(K[0] == Catch::Approx(0.057967f));
    CHECK(K[1] == Catch::Approx(128.044016f));
}

TEST_CASE("lqr_lut_eval interpolates between entries", "[lqr][lut]")
{
    float K[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    const float x0 = 0.418006f;
    const float x1 = 0.525897f;
    const float mid = 0.5f * (x0 + x1);
    REQUIRE(lqr_lut_eval(mid, K));

    const float k0 = -0.000201f;
    const float k1 = -0.000515f;
    const float k2 = -107.414622f;
    const float k3 = 95.762439f;
    CHECK(K[0] == Catch::Approx(0.5f * (k0 + k1)));
    CHECK(K[1] == Catch::Approx(0.5f * (k2 + k3)));
}
