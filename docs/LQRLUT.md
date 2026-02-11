# LQR LUT Integration Plan

This document describes how to integrate the LQR gain look‑up table (LUT) into the firmware so gains are scheduled by hip angle, and includes a ready‑to‑use C header generated from `lqr_lut.csv`.

## Plan (firmware route)

1) **Create LUT module**  
   - New files: `lqr_lut.h` / `lqr_lut.c` (or `.cpp` if preferred).  
   - Store the LUT arrays (hip angles + K gains).  
   - Provide:
     ```c
     bool lqr_lut_eval(float hip_rad, float K_out[4]);
     ```
     which linearly interpolates between the two nearest hip entries and clamps to endpoints.

2) **Read hip angle**  
   - Use existing hip state in `motion_control.cpp` via `hip_control_get_state(...)`.  
   - Use average left/right when both valid.

3) **Update controller gains**  
   - At a low rate (e.g., 20–50 Hz), call `lqr_lut_eval` and update `lqr_params_t.K[]` through `MotionController::setLqrParams`.
   - Keep `u_limit`, `du_limit`, etc. unchanged.

4) **Units**  
   - LUT gains are compatible with **torque** LQR (N·m) as used in `computeLqrUSumNm`.

---

## C header generated from `lqr_lut.csv`

```c
#ifndef LQR_LUT_DATA_H
#define LQR_LUT_DATA_H

#define LQR_LUT_SIZE 7

static const float LQR_HIP_LUT[LQR_LUT_SIZE] = {
    0.418006f,
    0.525897f,
    0.633787f,
    0.741678f,
    0.849568f,
    0.957459f,
    1.065349f
};

static const float LQR_K_LUT[LQR_LUT_SIZE][4] = {
    { -0.000201f, -107.414622f, -37194.881102f,  -953.895737f },
    { -0.000515f,   95.762439f, -10124.929687f,  -927.288576f },
    { -0.000820f,   92.016074f,  -5808.652021f,  -854.058594f },
    {  0.000054f,   91.636974f,  -2661.611492f,  -796.747432f },
    { -0.004077f,   91.990429f,  -7395.936836f,  -747.523630f },
    { -0.011577f,   38.871373f, -14390.611498f,  -693.586946f },
    {  0.057967f,  128.044016f, -45223.800408f,  -645.138548f }
};

#endif /* LQR_LUT_DATA_H */
```

