#include "hip_limits.h"

#include <stddef.h>

void hip_limit_init(hip_limit_debounce_t *state, uint8_t samples_required, uint8_t initial)
{
    if (state == NULL) {
        return;
    }
    state->stable = initial ? 1U : 0U;
    state->candidate = state->stable;
    state->count = 0U;
    state->samples_required = (samples_required == 0U) ? 1U : samples_required;
}

uint8_t hip_limit_update(hip_limit_debounce_t *state, uint8_t raw)
{
    if (state == NULL) {
        return 0U;
    }
    uint8_t sample = raw ? 1U : 0U;
    if (sample == state->candidate) {
        if (state->count < 0xFFU) {
            state->count++;
        }
    } else {
        state->candidate = sample;
        state->count = 1U;
    }

    if (state->count >= state->samples_required && state->stable != state->candidate) {
        state->stable = state->candidate;
    }
    return state->stable;
}

uint8_t hip_limit_get(const hip_limit_debounce_t *state)
{
    if (state == NULL) {
        return 0U;
    }
    return state->stable;
}
