#ifndef CONTROL_STATE_ESTIMATE_H
#define CONTROL_STATE_ESTIMATE_H

struct StateEstimate {
    float theta;
    float thetaDot;
    float x;
    float xDot;
    float gyroBias;
    bool valid;
};

#endif /* CONTROL_STATE_ESTIMATE_H */
