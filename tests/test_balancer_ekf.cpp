#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>

#include "ekf/BalancerEKF.h"

TEST_CASE("BalancerEKF step is stable under zero motion", "[balancer_ekf]")
{
    BalancerEKF ekf;
    ekf.begin();

    bool ok = ekf.step(0.0f, 0.0f, 0.0f,
                       0.0f, 0.0f, 0.0f,
                       0.01f, NAN);

    CHECK(ok);
    CHECK(ekf.isStateValid());

    BalancerState st = ekf.getState();
    CHECK(st.theta == Catch::Approx(0.0f).margin(1e-3f));
    CHECK(st.thetaDot == Catch::Approx(0.0f).margin(1e-3f));
    CHECK(st.x == Catch::Approx(0.0f).margin(1e-3f));
    CHECK(st.xDot == Catch::Approx(0.0f).margin(1e-3f));
}
