#ifndef CONTROL_HIP_LIMITS_H
#define CONTROL_HIP_LIMITS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t stable;
    uint8_t candidate;
    uint8_t count;
    uint8_t samples_required;
} hip_limit_debounce_t;

void hip_limit_init(hip_limit_debounce_t *state, uint8_t samples_required, uint8_t initial);
uint8_t hip_limit_update(hip_limit_debounce_t *state, uint8_t raw);
uint8_t hip_limit_get(const hip_limit_debounce_t *state);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_HIP_LIMITS_H */
