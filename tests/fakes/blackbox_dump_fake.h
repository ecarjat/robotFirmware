#ifndef TESTS_FAKES_BLACKBOX_DUMP_FAKE_H
#define TESTS_FAKES_BLACKBOX_DUMP_FAKE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void blackbox_dump_fake_reset(void);
uint32_t blackbox_dump_fake_capture_requests(void);
uint32_t blackbox_dump_fake_tick_calls(void);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_FAKES_BLACKBOX_DUMP_FAKE_H */
