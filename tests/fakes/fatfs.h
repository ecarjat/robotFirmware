#ifndef TESTS_FAKES_FATFS_H
#define TESTS_FAKES_FATFS_H

#include <stdint.h>

typedef uint32_t UINT;

typedef enum {
  FR_OK = 0,
  FR_DISK_ERR = 1,
  FR_INT_ERR = 2
} FRESULT;

typedef struct {
  uint32_t reserved;
} FIL;

typedef struct {
  uint32_t reserved;
} DIR;

typedef struct {
  char fname[256];
} FILINFO;

#define FA_WRITE 0x02U
#define FA_CREATE_ALWAYS 0x08U

extern uint8_t retSD;

FRESULT f_open(FIL *fp, const char *path, uint8_t mode);
FRESULT f_write(FIL *fp, const void *buffer, UINT bytes_to_write,
                UINT *bytes_written);
FRESULT f_sync(FIL *fp);
FRESULT f_close(FIL *fp);
FRESULT f_unlink(const char *path);
FRESULT f_opendir(DIR *dp, const char *path);
FRESULT f_readdir(DIR *dp, FILINFO *fno);
FRESULT f_closedir(DIR *dp);

#endif /* TESTS_FAKES_FATFS_H */
