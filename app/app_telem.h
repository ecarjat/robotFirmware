#ifndef APP_TELEM_H
#define APP_TELEM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void app_telem_init(void);
void app_telem_tick(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_TELEM_H */