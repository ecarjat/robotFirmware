#include <cmath>
#include <iostream>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

extern "C" {
#include "hip_kinematics.h"
}

namespace {
constexpr float kDegToRad = static_cast<float>(M_PI / 180.0);
}

TEST_CASE("hip_kinematics_height_from_theta returns valid height", "[hip]") {
  float height_m = 0.0f;

  REQUIRE(hip_kinematics_height_from_theta(23.95f * kDegToRad, &height_m));
  std::cout << "height @23.95deg = " << height_m << "\n";
  CHECK(height_m == Catch::Approx(0.2895f).margin(0.005f));

  REQUIRE(hip_kinematics_height_from_theta(61.04f * kDegToRad, &height_m));
  std::cout << "height @61.04deg = " << height_m << "\n";
  CHECK(height_m == Catch::Approx(0.6775f).margin(0.005f));
}

TEST_CASE("hip height increases with theta in operating range", "[hip]") {
  float h1 = 0.0f;
  float h2 = 0.0f;
  float h3 = 0.0f;

  REQUIRE(hip_kinematics_height_from_theta(24.0f * kDegToRad, &h1));
  REQUIRE(hip_kinematics_height_from_theta(40.0f * kDegToRad, &h2));
  REQUIRE(hip_kinematics_height_from_theta(61.0f * kDegToRad, &h3));

  std::cout << "height @24deg = " << h1 << "\n";
  std::cout << "height @40deg = " << h2 << "\n";
  std::cout << "height @61deg = " << h3 << "\n";
  CHECK(h1 < h2);
  CHECK(h2 < h3);
}

TEST_CASE("hip_kinematics_theta_from_height inverts height", "[hip]") {
  float theta_rad = 0.0f;
  float height_m = 0.0f;

  REQUIRE(hip_kinematics_theta_from_height(0.2895f, &theta_rad));
  CHECK(theta_rad == Catch::Approx(23.95f * kDegToRad).margin(0.01f));
  REQUIRE(hip_kinematics_height_from_theta(theta_rad, &height_m));
  CHECK(height_m == Catch::Approx(0.2895f).margin(0.005f));

  REQUIRE(hip_kinematics_theta_from_height(0.6775f, &theta_rad));
  CHECK(theta_rad == Catch::Approx(61.04f * kDegToRad).margin(0.01f));
  REQUIRE(hip_kinematics_height_from_theta(theta_rad, &height_m));
  CHECK(height_m == Catch::Approx(0.6775f).margin(0.005f));
}
