#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "StateEstimator.h"
#include "types.h"

TEST_CASE("StateEstimator math helpers", "[state_estimator]")
{
    float rot[9];
    state_estimator_test_set_identity(rot);

    CHECK(rot[0] == Catch::Approx(1.0f));
    CHECK(rot[4] == Catch::Approx(1.0f));
    CHECK(rot[8] == Catch::Approx(1.0f));

    float in[3] = {1.0f, -2.0f, 0.5f};
    float out[3] = {};
    state_estimator_test_apply_rotation(rot, in, out);
    CHECK(out[0] == Catch::Approx(in[0]));
    CHECK(out[1] == Catch::Approx(in[1]));
    CHECK(out[2] == Catch::Approx(in[2]));

    float v[3] = {3.0f, 4.0f, 0.0f};
    CHECK(state_estimator_test_vec_norm(v) == Catch::Approx(5.0f));

    float a[3] = {1.0f, 2.0f, 3.0f};
    float b[3] = {4.0f, 5.0f, 6.0f};
    CHECK(state_estimator_test_vec_dot(a, b) == Catch::Approx(32.0f));
}

TEST_CASE("StateEstimator update with simple IMU data yields valid estimate", "[state_estimator]")
{
    RobotParams params;
    StateEstimator estimator;
    REQUIRE(estimator.begin(params));

    ImuReading primary{};
    primary.timestamp_ms = 10U;
    primary.valid = true;
    primary.accel_x = 0.0f;
    primary.accel_y = 0.0f;
    primary.accel_z = 9.80665f;
    primary.gyro_x = 0.0f;
    primary.gyro_y = 0.0f;
    primary.gyro_z = 0.0f;

    ImuReading secondary = primary;
    secondary.timestamp_ms = 10U;
    secondary.valid = false;

    estimator.update(primary, secondary, 10U, 0.0f, 0.0f);
    StateEstimate est = estimator.getEstimate();

    CHECK(est.valid);
    CHECK(est.theta == Catch::Approx(0.0f).margin(0.05f));
}
