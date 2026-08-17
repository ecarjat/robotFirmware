#ifndef TESTS_FAKES_FATFS_FAKE_H
#define TESTS_FAKES_FATFS_FAKE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  FATFS_FAKE_FAIL_NONE = 0,
  FATFS_FAKE_FAIL_OPEN,
  FATFS_FAKE_FAIL_WRITE,
  FATFS_FAKE_FAIL_SYNC,
  FATFS_FAKE_FAIL_CLOSE
} fatfs_fake_failure_t;

void fatfs_fake_reset(void);
void fatfs_fake_set_failure(fatfs_fake_failure_t failure);
uint32_t fatfs_fake_open_calls(void);
uint32_t fatfs_fake_write_calls(void);
uint32_t fatfs_fake_sync_calls(void);
uint32_t fatfs_fake_close_calls(void);
uint32_t fatfs_fake_unlink_calls(void);
uint32_t fatfs_fake_opendir_calls(void);
uint32_t fatfs_fake_readdir_calls(void);
const uint8_t *fatfs_fake_file_data(void);
size_t fatfs_fake_file_size(void);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_FAKES_FATFS_FAKE_H */
