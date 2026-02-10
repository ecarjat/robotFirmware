#include "led_status.h"

static uint32_t s_flags = 0U;

void led_status_init(void)
{
    s_flags = 0U;
}

void led_status_update(uint32_t now_ms)
{
    (void)now_ms;
}

void led_status_set_flag(led_status_flags_t flag)
{
    s_flags |= (uint32_t)flag;
}

void led_status_clear_flag(led_status_flags_t flag)
{
    s_flags &= ~(uint32_t)flag;
}

void led_status_clear_all_flags(void)
{
    s_flags = 0U;
}

uint32_t led_status_get_flags(void)
{
    return s_flags;
}

void led_status_set_green_override(led_pattern_t pattern)
{
    (void)pattern;
}
