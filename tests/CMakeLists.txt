cmake_minimum_required(VERSION 3.22)

project(stm32Controller_host_tests LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(USE_FETCHCONTENT_CATCH2 "Fetch Catch2 if not found" ON)

include(FetchContent)
find_package(Catch2 3 QUIET)

if(NOT Catch2_FOUND AND USE_FETCHCONTENT_CATCH2)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.5.4
  )
  FetchContent_MakeAvailable(Catch2)
endif()

if(NOT Catch2_FOUND AND NOT TARGET Catch2::Catch2WithMain)
  message(FATAL_ERROR "Catch2 v3 not found. Install it or enable USE_FETCHCONTENT_CATCH2.")
endif()

enable_testing()

add_executable(test_hip_kinematics
  test_hip_kinematics.cpp
  ../app/control/hip_kinematics.c
)

target_include_directories(test_hip_kinematics PRIVATE
  ../app/control
)

target_link_libraries(test_hip_kinematics PRIVATE Catch2::Catch2WithMain)

add_test(NAME test_hip_kinematics COMMAND test_hip_kinematics)

add_executable(test_motion_modes
  test_motion_modes.cpp
  fakes/log_fake.c
  fakes/blackbox_dump_fake.c
  fakes/hal_fake.c
  ../app/control/motion_modes.cpp
)

target_include_directories(test_motion_modes PRIVATE
  fakes
  ../app/control
  ../app
  ../app/utils
  ../app/drivers
  ../app/logging
)

target_link_libraries(test_motion_modes PRIVATE Catch2::Catch2WithMain)

add_test(NAME test_motion_modes COMMAND test_motion_modes)

add_executable(test_control_timer
  test_control_timer.cpp
  fakes/hal_fake.c
  fakes/log_fake.c
  ../app/control/control_timer.c
)

target_include_directories(test_control_timer PRIVATE
  fakes
  ../app/control
  ../app/utils
  ../app
)

target_compile_definitions(test_control_timer PRIVATE CONTROL_TIMER_TEST=1)

target_link_libraries(test_control_timer PRIVATE Catch2::Catch2WithMain)

add_test(NAME test_control_timer COMMAND test_control_timer)

add_executable(test_motion_controller
  test_motion_controller.cpp
  fakes/log_fake.c
  ../app/control/MotionController.cpp
)

target_include_directories(test_motion_controller PRIVATE
  fakes
  ../app/control
  ../app/utils
  ../app/drivers
) 

target_compile_definitions(test_motion_controller PRIVATE UNIT_TEST=1)

target_link_libraries(test_motion_controller PRIVATE Catch2::Catch2WithMain)

add_test(NAME test_motion_controller COMMAND test_motion_controller)

add_executable(test_state_estimator
  test_state_estimator.cpp
  fakes/log_fake.c
  ../app/control/StateEstimator.cpp
  ../app/control/ekf/BalancerEKF.cpp
)

target_include_directories(test_state_estimator PRIVATE
  fakes
  ../app/control
  ../app/utils
  ../app/drivers
  ../libs/TinyEKF
)

target_compile_definitions(test_state_estimator PRIVATE UNIT_TEST=1)

target_link_libraries(test_state_estimator PRIVATE Catch2::Catch2WithMain)

add_test(NAME test_state_estimator COMMAND test_state_estimator)

add_executable(test_balancer_ekf
  test_balancer_ekf.cpp
  ../app/control/ekf/BalancerEKF.cpp
)

target_include_directories(test_balancer_ekf PRIVATE
  fakes
  ../app/control
  ../libs/TinyEKF
)

target_link_libraries(test_balancer_ekf PRIVATE Catch2::Catch2WithMain)

add_test(NAME test_balancer_ekf COMMAND test_balancer_ekf)

add_executable(test_motion_control
  test_motion_control.cpp
  fakes/log_fake.c
  fakes/blackbox_fake.c
  fakes/blackbox_dump_fake.c
  fakes/crc32_fake.c
  fakes/imu_fake.c
  fakes/motor_link_fake.c
  fakes/app_main_fake.c
  ../app/control/motion_control.cpp
  ../app/control/motion_modes.cpp
  ../app/control/MotionController.cpp
  ../app/control/StateEstimator.cpp
  ../app/control/ekf/BalancerEKF.cpp
)

target_include_directories(test_motion_control PRIVATE
  fakes
  ../app/control
  ../app
  ../app/utils
  ../app/drivers
  ../app/drivers/imu
  ../app/drivers/motors
  ../app/logging
  ../libs/TinyEKF
)

target_compile_definitions(test_motion_control PRIVATE UNIT_TEST=1)

target_link_libraries(test_motion_control PRIVATE Catch2::Catch2WithMain)

add_test(NAME test_motion_control COMMAND test_motion_control)
