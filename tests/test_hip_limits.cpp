#include <catch2/catch_test_macros.hpp>

extern "C" {
#include "hip_limits.h"
}

TEST_CASE("hip_limit debounce requires consecutive samples", "[hip_limits]")
{
    hip_limit_debounce_t sw;
    hip_limit_init(&sw, 3, 0);

    CHECK(hip_limit_update(&sw, 1) == 0);
    CHECK(hip_limit_update(&sw, 1) == 0);
    CHECK(hip_limit_update(&sw, 1) == 1);

    CHECK(hip_limit_update(&sw, 0) == 1);
    CHECK(hip_limit_update(&sw, 0) == 1);
    CHECK(hip_limit_update(&sw, 0) == 0);
}

TEST_CASE("hip_limit debounce ignores rapid bouncing", "[hip_limits]")
{
    hip_limit_debounce_t sw;
    hip_limit_init(&sw, 4, 0);

    CHECK(hip_limit_update(&sw, 1) == 0);
    CHECK(hip_limit_update(&sw, 0) == 0);
    CHECK(hip_limit_update(&sw, 1) == 0);
    CHECK(hip_limit_update(&sw, 0) == 0);
    CHECK(hip_limit_update(&sw, 1) == 0);
}
