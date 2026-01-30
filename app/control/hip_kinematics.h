#ifndef HIP_KINEMATICS_H
#define HIP_KINEMATICS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Compute wheel/ground height (hip to wheel distance, +Y down) from hip angle.
 *
 * Coordinate frame matches the solver:
 * - Hip is at (0,0)
 * - +X is forward, +Y is downward
 * - Theta is the hip angle of the upper leg (H-K) from +X axis, in radians.
 *
 * @param theta_rad Hip angle in radians.
 * @param height_m Output height in meters (positive down).
 * @return true if geometry is solvable, false if no valid intersection exists.
 */
bool hip_kinematics_height_from_theta(float theta_rad, float *height_m);

/**
 * @brief Compute hip angle from height using a lookup table + interpolation.
 *
 * Uses a precomputed LUT derived from hip_kinematics_height_from_theta across
 * the valid theta range.
 *
 * @param height_m Desired height in meters (positive down).
 * @param theta_rad Output hip angle in radians.
 * @return true if height is in range and interpolation succeeds.
 */
bool hip_kinematics_theta_from_height(float height_m, float *theta_rad);

#ifdef __cplusplus
}
#endif

#endif /* HIP_KINEMATICS_H */
