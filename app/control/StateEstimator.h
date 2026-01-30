#ifndef CONTROL_STATE_ESTIMATOR_H
#define CONTROL_STATE_ESTIMATOR_H

#include <stdint.h>

#include "RobotParams.h"
#include "StateEstimate.h"
#include "config_control.h"
#include "ekf/BalancerEKF.h"
#include "types.h"

struct EkfLogData {
    bool valid;
    float dt_s;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_norm_g;
    float theta;
    float theta_acc;
    float gyro_rate;
    uint32_t imu_primary_dt_ms;
    uint32_t imu_secondary_dt_ms;
    float innov;
    float s;
    float k0;
    float k1;
    float p00;
    float p01;
    float p11;
    float r_used;
    uint8_t gate;
    uint8_t still;
};

struct ImuHealthMetrics {
    bool valid;
    uint8_t active_sensor; /* 0 = primary (BMI270), 1 = secondary (ICM42688) */
    float gyro_diff_dps;
    float gyro_pitch_diff_dps;
    float acc_angle_diff_deg;
    float vib_rms_g;
    uint8_t gate_accel;
};

class StateEstimator {
public:
    StateEstimator();

    bool begin(const RobotParams &params);
    void update(const ImuReading &primary, const ImuReading &secondary,
                uint32_t now_ms, float v_enc, float yaw_rate_enc);
    void setControlDt(float dtSeconds);
    float getEstimatedYawRate() const;
    void setImuRotations(const float *primary, const float *secondary);

    StateEstimate getEstimate() const;
    bool getEkfLogData(EkfLogData& out) const;
    bool getImuHealthMetrics(ImuHealthMetrics& out) const;
    void resetTiming();

    float getLastThetaAcc() const { return lastThetaAcc_; }
    float getLastEncVelocity() const { return lastVEnc_; }
    float getLastGyroPitch() const { return lastGyroPitch_; }
    float getLastGyroZ() const { return ekf_log_data_.gyro_z; }

private:
    BalancerEKF ekf_;
    StateEstimate estimate_;

    uint32_t lastUpdateMs_;

    bool bootInitDone_;

    float lastThetaAcc_;
    float lastVEnc_;
    float lastGyroPitch_;
    float lastWheelAngleL_;
    float lastWheelAngleR_;
    float lastWheelMechL_;
    float lastWheelMechR_;
    bool haveWheelAngles_;
    float odomX_;

    uint32_t last_primary_ts_;
    uint32_t last_secondary_ts_;

    int initSampleCount_;
    float initPitchSum_;

    float wheelRadius_;
    float control_dt_;

    bool initialized_;

    EkfLogData ekf_log_data_;
    bool ekf_log_valid_;

    ImuHealthMetrics imu_health_;
    bool imu_health_valid_;

    float imu_primary_rot_[9];
    float imu_secondary_rot_[9];

    struct VibWindow {
        float samples[IMU_VIB_WINDOW_SAMPLES];
        uint32_t count;
        uint32_t index;
        float sum_sq;
        float rms;
    };

    VibWindow vib_primary_;
    VibWindow vib_secondary_;

    bool use_secondary_;
    uint32_t last_switch_ms_;
    uint32_t primary_unhealthy_since_ms_;
    uint32_t primary_healthy_since_ms_;
};

#ifdef UNIT_TEST
void state_estimator_test_set_identity(float rot[9]);
void state_estimator_test_apply_rotation(const float rot[9], const float in[3], float out[3]);
float state_estimator_test_vec_norm(const float v[3]);
float state_estimator_test_vec_dot(const float a[3], const float b[3]);
#endif

#endif /* CONTROL_STATE_ESTIMATOR_H */
