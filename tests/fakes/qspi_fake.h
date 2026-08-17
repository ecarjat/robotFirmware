#ifndef TESTS_FAKES_QSPI_FAKE_H
#define TESTS_FAKES_QSPI_FAKE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void qspi_fake_reset(void);
void qspi_fake_power_cycle(void);
void qspi_fake_reset_counters(void);
void qspi_fake_set_read_failure(int enabled);
void qspi_fake_fail_next_read(void);
void qspi_fake_power_loss_during_next_erase(void);
void qspi_fake_power_loss_during_next_write(size_t programmed_bytes);
uint32_t qspi_fake_read_calls(void);
uint32_t qspi_fake_write_calls(void);
uint32_t qspi_fake_erase_calls(void);
uint32_t qspi_fake_last_write_addr(void);
uint32_t qspi_fake_last_erase_addr(void);
const uint8_t *qspi_fake_data(void);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_FAKES_QSPI_FAKE_H */
