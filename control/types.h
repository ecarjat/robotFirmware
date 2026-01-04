#ifndef CONTROL_TYPES_H
#define CONTROL_TYPES_H

#include <stdbool.h>
#include <stdint.h>

struct ImuReading {
    uint32_t timestamp_ms;
    float pitch_rad;
    float roll_rad;
    float yaw_rad;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    bool valid;
};

struct DistanceReading {
    float front_m;
    float rear_m;
    bool valid_front;
    bool valid_rear;
};

struct MotionCommand {
    float forward_velocity;
    float turn_rate;
    bool motors_enabled;
    bool menu_enter;
    bool menu_back;
    bool menu_next;
    bool menu_prev;
    bool menu_toggle;
    float menu_nav;
};

#endif /* CONTROL_TYPES_H */
