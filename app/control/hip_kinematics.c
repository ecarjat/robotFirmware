#include "hip_kinematics.h"

#include <math.h>
#include <stddef.h>


typedef struct {
  float x;
  float y;
} hip_point_t;

/* Geometry parameters (meters). Values derived from solver selection (mm). */
#define HIP_UPPER_LEG_M 0.39609f
#define HIP_LOWER_LEG_M 0.37996f
#define HIP_LINK_BC_C_M 0.41762f
#define HIP_INNER_OFFSET_KC_M 0.07506f

/* Theta range (degrees) for valid linkage motion. */
#define HIP_DEG_TO_RAD ((float)(M_PI / 180.0))

/* Pin joint location in solver coordinates (+Y down). */
#define HIP_PIN_X_M 0.09893f
#define HIP_PIN_Y_M (-0.11576f)

#define HIP_EPS 1.0e-7f
#define HIP_LUT_HEIGHT_TOL 1.0e-3f

/* Lookup table resolution for inverse mapping (height -> theta). */
#define HIP_LUT_SIZE 129

static float hip_dist(const hip_point_t *a, const hip_point_t *b) {
  const float dx = b->x - a->x;
  const float dy = b->y - a->y;
  return sqrtf(dx * dx + dy * dy);
}

static bool hip_circle_intersections(const hip_point_t *c0, float r0,
                                     const hip_point_t *c1, float r1,
                                     hip_point_t *p3,
                                     hip_point_t *p4) {
  const float dx = c1->x - c0->x;
  const float dy = c1->y - c0->y;
  const float d = sqrtf(dx * dx + dy * dy);

  if (d < HIP_EPS) {
    return false;
  }

  if (d > (r0 + r1 + HIP_EPS)) {
    return false;
  }

  if (d < (fabsf(r0 - r1) - HIP_EPS)) {
    return false;
  }

  const float a = (r0 * r0 - r1 * r1 + d * d) / (2.0f * d);
  float h_sq = r0 * r0 - a * a;
  if (h_sq < 0.0f) {
    if (h_sq < -HIP_EPS) {
      return false;
    }
    h_sq = 0.0f;
  }
  const float h = sqrtf(h_sq);

  const float x2 = c0->x + a * dx / d;
  const float y2 = c0->y + a * dy / d;
  const float rx = -dy * (h / d);
  const float ry = dx * (h / d);

  p3->x = x2 + rx;
  p3->y = y2 + ry;
  p4->x = x2 - rx;
  p4->y = y2 - ry;
  return true;
}

static hip_point_t hip_compute_wheel(const hip_point_t *k, const hip_point_t *c,
                                     float lkw) {
  const float vx = c->x - k->x;
  const float vy = c->y - k->y;
  const float n = sqrtf(vx * vx + vy * vy);
  hip_point_t w = *k;

  if (n < HIP_EPS) {
    w.y += lkw;
    return w;
  }

  const float ux = vx / n;
  const float uy = vy / n;
  w.x = k->x - ux * lkw;
  w.y = k->y - uy * lkw;
  return w;
}

bool hip_kinematics_height_from_theta(float theta_rad, float *height_m) {
  if (height_m == NULL) {
    return false;
  }

  const hip_point_t bc = {HIP_PIN_X_M, HIP_PIN_Y_M};

  hip_point_t k;
  k.x = HIP_UPPER_LEG_M * cosf(theta_rad);
  k.y = HIP_UPPER_LEG_M * sinf(theta_rad);

  hip_point_t c1, c2;
  if (!hip_circle_intersections(&k, HIP_INNER_OFFSET_KC_M, &bc, HIP_LINK_BC_C_M,
                                &c1, &c2)) {
    return false;
  }

  const hip_point_t w1 = hip_compute_wheel(&k, &c1, HIP_LOWER_LEG_M);
  const hip_point_t w2 = hip_compute_wheel(&k, &c2, HIP_LOWER_LEG_M);

  /* Choose the branch with larger Y (downward height). */
  const hip_point_t w = (w1.y >= w2.y) ? w1 : w2;

  *height_m = w.y;
  return true;
}

static bool hip_lut_initialized = false;
static bool hip_lut_valid = false;
static float hip_theta_lut[HIP_LUT_SIZE];
static float hip_height_lut[HIP_LUT_SIZE];
static bool hip_height_increasing = true;
static bool hip_height_monotonic = true;
static float hip_height_min = 0.0f;
static float hip_height_max = 0.0f;

static void hip_init_lut(void) {
  if (hip_lut_initialized) {
    return;
  }

  const float theta_min = HIP_THETA_MIN_DEG * HIP_DEG_TO_RAD;
  const float theta_max = HIP_THETA_MAX_DEG * HIP_DEG_TO_RAD;
  const float step = (theta_max - theta_min) / (HIP_LUT_SIZE - 1);

  hip_lut_valid = false;
  hip_height_monotonic = true;
  hip_height_min = 0.0f;
  hip_height_max = 0.0f;
  bool have_min_max = false;
  int valid_count = 0;

  for (int i = 0; i < HIP_LUT_SIZE; ++i) {
    const float theta = theta_min + step * i;
    hip_theta_lut[i] = theta;
    if (!hip_kinematics_height_from_theta(theta, &hip_height_lut[i])) {
      hip_height_lut[i] = NAN;
      continue;
    }

    ++valid_count;
    if (!have_min_max) {
      hip_height_min = hip_height_lut[i];
      hip_height_max = hip_height_lut[i];
      have_min_max = true;
    } else {
      if (hip_height_lut[i] < hip_height_min) {
        hip_height_min = hip_height_lut[i];
      }
      if (hip_height_lut[i] > hip_height_max) {
        hip_height_max = hip_height_lut[i];
      }
    }

    if (i > 0 && !isnan(hip_height_lut[i - 1])) {
      const float d = hip_height_lut[i] - hip_height_lut[i - 1];
      if (i == 1) {
        hip_height_increasing = (d >= 0.0f);
      } else if ((d >= 0.0f) != hip_height_increasing) {
        hip_height_monotonic = false;
      }
    }
  }

  if (have_min_max && valid_count >= 2) {
    hip_lut_valid = true;
  }
  hip_lut_initialized = true;
}

bool hip_kinematics_theta_from_height(float height_m, float *theta_rad) {
  if (theta_rad == NULL) {
    return false;
  }

  hip_init_lut();

  if (!hip_lut_valid) {
    return false;
  }

  if (height_m < hip_height_min - HIP_LUT_HEIGHT_TOL ||
      height_m > hip_height_max + HIP_LUT_HEIGHT_TOL) {
    return false;
  }

  if (height_m < hip_height_min) {
    height_m = hip_height_min;
  } else if (height_m > hip_height_max) {
    height_m = hip_height_max;
  }

  int lo = -1;
  int hi = -1;
  if (hip_height_monotonic) {
    lo = 0;
    hi = HIP_LUT_SIZE - 1;
    while ((hi - lo) > 1) {
      const int mid = (lo + hi) / 2;
      const float h_mid = hip_height_lut[mid];

      const bool less_than = hip_height_increasing ? (h_mid < height_m)
                                                   : (h_mid > height_m);
      if (less_than) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
  } else {
    for (int i = 0; i < HIP_LUT_SIZE - 1; ++i) {
      const float h_a = hip_height_lut[i];
      const float h_b = hip_height_lut[i + 1];
      if (isnan(h_a) || isnan(h_b)) {
        continue;
      }
      if ((height_m >= h_a && height_m <= h_b) ||
          (height_m >= h_b && height_m <= h_a)) {
        lo = i;
        hi = i + 1;
        break;
      }
    }
    if (lo < 0 || hi < 0) {
      return false;
    }
  }

  const float h_lo = hip_height_lut[lo];
  const float h_hi = hip_height_lut[hi];
  if (isnan(h_lo) || isnan(h_hi)) {
    return false;
  }
  const float t = (fabsf(h_hi - h_lo) < HIP_EPS) ? 0.0f
                                                 : (height_m - h_lo) / (h_hi - h_lo);
  *theta_rad = hip_theta_lut[lo] + t * (hip_theta_lut[hi] - hip_theta_lut[lo]);
  return true;
}

bool hip_kinematics_get_height_range(float *min_m, float *max_m)
{
  hip_init_lut();
  if (!hip_lut_valid || min_m == NULL || max_m == NULL) {
    return false;
  }
  *min_m = hip_height_min;
  *max_m = hip_height_max;
  return true;
}
