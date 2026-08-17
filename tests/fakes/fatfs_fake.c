#include "fatfs.h"
#include "fatfs_fake.h"

#include <stdbool.h>
#include <string.h>

#define FATFS_FAKE_FILE_CAPACITY (64U * 1024U)

uint8_t retSD = 0U;

static uint8_t s_file_data[FATFS_FAKE_FILE_CAPACITY];
static size_t s_file_size;
static bool s_file_open;
static fatfs_fake_failure_t s_failure;
static uint32_t s_open_calls;
static uint32_t s_write_calls;
static uint32_t s_sync_calls;
static uint32_t s_close_calls;
static uint32_t s_unlink_calls;
static uint32_t s_opendir_calls;
static uint32_t s_readdir_calls;

void fatfs_fake_reset(void) {
  memset(s_file_data, 0, sizeof(s_file_data));
  s_file_size = 0U;
  s_file_open = false;
  s_failure = FATFS_FAKE_FAIL_NONE;
  s_open_calls = 0U;
  s_write_calls = 0U;
  s_sync_calls = 0U;
  s_close_calls = 0U;
  s_unlink_calls = 0U;
  s_opendir_calls = 0U;
  s_readdir_calls = 0U;
  retSD = 0U;
}

void fatfs_fake_set_failure(fatfs_fake_failure_t failure) {
  s_failure = failure;
}

uint32_t fatfs_fake_open_calls(void) { return s_open_calls; }
uint32_t fatfs_fake_write_calls(void) { return s_write_calls; }
uint32_t fatfs_fake_sync_calls(void) { return s_sync_calls; }
uint32_t fatfs_fake_close_calls(void) { return s_close_calls; }
uint32_t fatfs_fake_unlink_calls(void) { return s_unlink_calls; }
uint32_t fatfs_fake_opendir_calls(void) { return s_opendir_calls; }
uint32_t fatfs_fake_readdir_calls(void) { return s_readdir_calls; }
const uint8_t *fatfs_fake_file_data(void) { return s_file_data; }
size_t fatfs_fake_file_size(void) { return s_file_size; }

FRESULT f_open(FIL *fp, const char *path, uint8_t mode) {
  (void)fp;
  (void)path;
  (void)mode;
  s_open_calls++;
  if (s_failure == FATFS_FAKE_FAIL_OPEN) {
    return FR_DISK_ERR;
  }
  s_file_open = true;
  s_file_size = 0U;
  return FR_OK;
}

FRESULT f_write(FIL *fp, const void *buffer, UINT bytes_to_write,
                UINT *bytes_written) {
  (void)fp;
  s_write_calls++;
  if (bytes_written != NULL) {
    *bytes_written = 0U;
  }
  if (s_failure == FATFS_FAKE_FAIL_WRITE || !s_file_open ||
      bytes_to_write > FATFS_FAKE_FILE_CAPACITY - s_file_size) {
    return FR_DISK_ERR;
  }
  memcpy(s_file_data + s_file_size, buffer, bytes_to_write);
  s_file_size += bytes_to_write;
  if (bytes_written != NULL) {
    *bytes_written = bytes_to_write;
  }
  return FR_OK;
}

FRESULT f_sync(FIL *fp) {
  (void)fp;
  s_sync_calls++;
  return (s_failure == FATFS_FAKE_FAIL_SYNC) ? FR_DISK_ERR : FR_OK;
}

FRESULT f_close(FIL *fp) {
  (void)fp;
  s_close_calls++;
  s_file_open = false;
  return (s_failure == FATFS_FAKE_FAIL_CLOSE) ? FR_DISK_ERR : FR_OK;
}

FRESULT f_unlink(const char *path) {
  (void)path;
  s_unlink_calls++;
  s_file_size = 0U;
  return FR_OK;
}

FRESULT f_opendir(DIR *dp, const char *path) {
  (void)dp;
  (void)path;
  s_opendir_calls++;
  return FR_OK;
}

FRESULT f_readdir(DIR *dp, FILINFO *fno) {
  (void)dp;
  s_readdir_calls++;
  if (fno != NULL) {
    fno->fname[0] = '\0';
  }
  return FR_OK;
}

FRESULT f_closedir(DIR *dp) {
  (void)dp;
  return FR_OK;
}
