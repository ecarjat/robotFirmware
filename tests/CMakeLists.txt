cmake_minimum_required(VERSION 3.22)

project(stm32Controller_host_tests LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(USE_FETCHCONTENT_CATCH2 "Fetch Catch2 if not found" ON)

add_compile_definitions(APP_CONFIG_HOST)

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
  fakes
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

add_executable(test_hip_limits
  test_hip_limits.cpp
  ../app/control/hip_limits.c
)

target_include_directories(test_hip_limits PRIVATE
  ../app/control
)

target_link_libraries(test_hip_limits PRIVATE Catch2::Catch2WithMain)

add_test(NAME test_hip_limits COMMAND test_hip_limits)

add_executable(test_lqr_lut
  test_lqr_lut.cpp
  ../app/control/lqr_lut.c
)

target_include_directories(test_lqr_lut PRIVATE
  ../app/control
)

target_link_libraries(test_lqr_lut PRIVATE Catch2::Catch2WithMain)

add_test(NAME test_lqr_lut COMMAND test_lqr_lut)

add_executable(test_hip_behavior
  test_hip_behavior.cpp
  fakes/log_fake.c
  ../app/control/hip_behavior.c
)

target_include_directories(test_hip_behavior PRIVATE
  fakes
  ../app/control
  ../app/utils
  ../app
  ../app/drivers
)

target_link_libraries(test_hip_behavior PRIVATE Catch2::Catch2WithMain)

add_test(NAME test_hip_behavior COMMAND test_hip_behavior)

add_executable(test_hip_control
  test_hip_control.cpp
  fakes/hal_fake.c
  fakes/log_fake.c
  ../app/control/hip_control.c
  ../app/control/hip_limits.c
  ../app/control/hip_kinematics.c
)

target_include_directories(test_hip_control PRIVATE
  fakes
  ../app/control
  ../app/utils
  ../app
  ../app/drivers
)

target_link_libraries(test_hip_control PRIVATE Catch2::Catch2WithMain)

add_test(NAME test_hip_control COMMAND test_hip_control)

add_executable(test_hip_integration
  test_hip_integration.cpp
  fakes/hal_fake.c
  fakes/log_fake.c
  ../app/control/hip_behavior.c
  ../app/control/hip_control.c
  ../app/control/hip_limits.c
  ../app/control/hip_kinematics.c
)

target_include_directories(test_hip_integration PRIVATE
  fakes
  ../app/control
  ../app/utils
  ../app
  ../app/drivers
)

target_link_libraries(test_hip_integration PRIVATE Catch2::Catch2WithMain)

add_test(NAME test_hip_integration COMMAND test_hip_integration)

add_executable(test_motor_backend_steadywin_can
  test_motor_backend_steadywin_can.cpp
  fakes/hal_fake.c
  fakes/log_fake.c
  fakes/hip_control_fake.c
  ../app/control/hip_limits.c
  ../app/control/hip_kinematics.c
  ../app/drivers/motors/motor_backend_steadywin_can.c
)

target_include_directories(test_motor_backend_steadywin_can PRIVATE
  fakes
  ../app
  ../app/control
  ../app/utils
  ../app/drivers
  ../app/drivers/motors
)

target_link_libraries(test_motor_backend_steadywin_can PRIVATE Catch2::Catch2WithMain)

add_test(NAME test_motor_backend_steadywin_can COMMAND test_motor_backend_steadywin_can)

add_executable(test_motion_controller
  test_motion_controller.cpp
  fakes/log_fake.c
  fakes/motor_link_fake.c
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
  fakes/hal_fake.c
  ../app/control/hip_behavior.c
  ../app/control/hip_kinematics.c
  ../app/control/lqr_lut.c
  ../app/control/hip_control.c
  ../app/control/hip_limits.c
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

add_executable(test_app_telem
  test_app_telem.cpp
  fakes/app_link_fake.c
  fakes/app_adc_fake.c
  fakes/led_status_fake.c
  fakes/hip_control_fake.c
  fakes/motion_control_fake.c
  fakes/motor_link_fake.c
  fakes/blackbox_dump_fake.c
  fakes/app_main_fake.c
  fakes/hal_fake.c
  fakes/log_fake.c
  app_telem_wrapper.c
)

target_include_directories(test_app_telem PRIVATE
  fakes
  ../app
  ../app/utils
  ../app/control
  ../app/drivers
  ../app/drivers/motors
  ../app/logging
  ../common
  ../common/shared_protocol
)

target_link_libraries(test_app_telem PRIVATE Catch2::Catch2WithMain)

add_test(NAME test_app_telem COMMAND test_app_telem)

add_executable(test_app_rpc
  test_app_rpc.cpp
  fakes/app_link_fake.c
  fakes/app_motor_fake.c
  fakes/app_arm_fake.c
  fakes/app_imu_fake.c
  fakes/hip_control_fake.c
  fakes/param_storage_fake.c
  fakes/control_timer_fake.c
  fakes/motion_control_fake.c
  fakes/app_main_fake.c
  fakes/hal_fake.c
  fakes/log_fake.c
  app_rpc_wrapper.c
)

target_include_directories(test_app_rpc PRIVATE
  fakes
  ../app
  ../app/utils
  ../app/control
  ../app/drivers
  ../app/drivers/motors
  ../common
  ../common/shared_protocol
)

target_link_libraries(test_app_rpc PRIVATE Catch2::Catch2WithMain)

add_test(NAME test_app_rpc COMMAND test_app_rpc)

add_executable(test_blackbox_deferred_export
  test_blackbox_deferred_export.cpp
  fakes/fatfs_fake.c
  fakes/qspi_fake.c
  fakes/crc32_fake.c
  fakes/led_status_fake.c
  fakes/hal_fake.c
  fakes/log_fake.c
  ../app/logging/blackbox.c
  ../app/logging/blackbox_dump.c
)

target_include_directories(test_blackbox_deferred_export PRIVATE
  fakes
  ../app
  ../app/control
  ../app/drivers
  ../app/logging
  ../app/utils
)

target_link_libraries(test_blackbox_deferred_export PRIVATE Catch2::Catch2WithMain)

add_test(NAME test_blackbox_deferred_export COMMAND test_blackbox_deferred_export)

add_executable(test_blackbox_metadata
  test_blackbox_metadata.cpp
  fakes/qspi_fake.c
  fakes/crc32_fake.c
  fakes/led_status_fake.c
  fakes/hal_fake.c
  fakes/log_fake.c
  fakes/blackbox_dump_fake.c
  ../app/logging/blackbox.c
)

target_include_directories(test_blackbox_metadata PRIVATE
  fakes
  ../app
  ../app/control
  ../app/drivers
  ../app/logging
  ../app/utils
)

target_compile_definitions(test_blackbox_metadata PRIVATE BLACKBOX_TESTING=1)
target_link_libraries(test_blackbox_metadata PRIVATE Catch2::Catch2WithMain)

add_test(NAME test_blackbox_metadata COMMAND test_blackbox_metadata)
