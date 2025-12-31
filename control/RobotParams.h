#ifndef CONTROL_ROBOT_PARAMS_H
#define CONTROL_ROBOT_PARAMS_H

#include "config_control.h"

struct RobotParams {
    float wheelRadius;
    float wheelBase;
    float bodyMass;
    float wheelMass;
    float comHeight;
    float bodyInertia;
    float motorKt;
    float motorResistance;
    float gearRatio;
    float maxVoltage;
    float maxWheelTorque;
    float gravity;
    float maxForwardVelocity;
    float maxTurnTorque;

    RobotParams()
        : wheelRadius(PARAM_WHEEL_RADIUS),
          wheelBase(PARAM_WHEEL_BASE),
          bodyMass(PARAM_BODY_MASS),
          wheelMass(PARAM_WHEEL_MASS),
          comHeight(PARAM_COM_HEIGHT),
          bodyInertia(PARAM_BODY_INERTIA),
          motorKt(PARAM_MOTOR_KT),
          motorResistance(PARAM_MOTOR_RESISTANCE),
          gearRatio(PARAM_GEAR_RATIO),
          maxVoltage(PARAM_MAX_VOLTAGE),
          maxWheelTorque(PARAM_MAX_WHEEL_TORQUE),
          gravity(PARAM_GRAVITY),
          maxForwardVelocity(PARAM_MAX_FORWARD_VEL),
          maxTurnTorque(PARAM_MAX_TURN_TORQUE) {}
};

#endif /* CONTROL_ROBOT_PARAMS_H */
