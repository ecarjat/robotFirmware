#include "app_file.h"
#include "app_config.h"
#include "app_link.h"
#include "motion_control.h"
#include "robot_protocol.h"
#include "ff.h"
#include "fatfs.h"
#include <string.h>

/* File transfer state - only one operation at a time */
static struct {
    bool busy;
} s_file_ctx;

static FIL s_file_handle __attribute__((section(".dma_buffer"), aligned(32)));
static uint8_t s_file_resp_buf[ROBOT_FRAME_MAX_PAYLOAD]
    __attribute__((section(".dma_buffer"), aligned(32)));

/**
 * @brief Check if file transfer is safe (robot must not be balancing)
 * @return true if safe to transfer files
 */
static bool app_file_is_safe(void)
{
    motion_mode_t mode = motion_control_get_mode();
    /* Only allow file transfer when NOT balancing */
    return (mode != MOTION_MODE_BALANCING);
}

/**
 * @brief Send file error response
 */
static bool app_file_send_error(uint16_t seq, uint8_t error_code, const char *filename)
{
    robot_file_err_t err;
    err.error_code = error_code;
    memset(err.filename, 0, sizeof(err.filename));
    if (filename != NULL) {
        strncpy(err.filename, filename, sizeof(err.filename) - 1U);
    }

    return app_link_send(ROBOT_MSG_FILE_ERR, ROBOT_FLAG_ACK_REQ,
                         (const uint8_t *)&err, sizeof(err), seq);
}

/**
 * @brief Handle FILE_LIST_REQ - scan SD card and send file list
 */
static bool app_file_handle_list(uint16_t seq, const uint8_t *payload, uint16_t payload_len)
{
    (void)payload;
    (void)payload_len;

    /* Safety check */
    if (!app_file_is_safe()) {
        APP_LOG_WARN("File list rejected: robot is balancing");
        return app_file_send_error(seq, ROBOT_FILE_ERR_BUSY, "");
    }

    /* Open root directory */
    DIR dir;
    FILINFO fno;
    FRESULT res = f_opendir(&dir, "/");
    if (res != FR_OK) {
        APP_LOG_ERROR("Failed to open SD root directory (err=%d)", res);
        return app_file_send_error(seq, ROBOT_FILE_ERR_READ_ERROR, "");
    }

    /* Build response - can send multiple file entries per frame */
    uint8_t resp_buf[ROBOT_FRAME_MAX_PAYLOAD];
    robot_file_list_resp_hdr_t *hdr = (robot_file_list_resp_hdr_t *)resp_buf;
    hdr->count = 0;
    hdr->more = 0;
    size_t offset = sizeof(robot_file_list_resp_hdr_t);

    /* Scan directory and add entries */
    while (true) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) {
            break;  /* End of directory or error */
        }

        /* Skip directories and hidden files */
        if (fno.fattrib & AM_DIR) {
            continue;
        }
        if (fno.fname[0] == '.') {
            continue;
        }

        /* Check if entry fits in frame */
        if (offset + sizeof(robot_file_entry_t) > sizeof(resp_buf)) {
            hdr->more = 1;  /* More files exist but don't fit in this frame */
            break;
        }

        /* Add file entry */
        robot_file_entry_t *entry = (robot_file_entry_t *)(resp_buf + offset);
        memset(entry->filename, 0, sizeof(entry->filename));
        strncpy(entry->filename, fno.fname, sizeof(entry->filename) - 1U);
        entry->size = fno.fsize;

        hdr->count++;
        offset += sizeof(robot_file_entry_t);
    }

    f_closedir(&dir);

    APP_LOG_INFO("File list: %u files", (unsigned int)hdr->count);

    return app_link_send(ROBOT_MSG_FILE_LIST_RESP, ROBOT_FLAG_ACK_REQ,
                         resp_buf, (uint16_t)offset, seq);
}

/**
 * @brief Handle FILE_READ_REQ - read chunk from file
 */
static bool app_file_handle_read(uint16_t seq, const uint8_t *payload, uint16_t payload_len)
{
    if (payload_len < sizeof(robot_file_read_req_t)) {
        APP_LOG_ERROR("FILE_READ_REQ too short");
        return app_file_send_error(seq, ROBOT_FILE_ERR_INVALID_REQ, "");
    }

    const robot_file_read_req_t *req = (const robot_file_read_req_t *)payload;

    /* Safety check */
    if (!app_file_is_safe()) {
        APP_LOG_WARN("File read rejected: robot is balancing");
        return app_file_send_error(seq, ROBOT_FILE_ERR_BUSY, req->filename);
    }

    /* Validate request */
    uint16_t req_len = req->length;
    if (req_len == 0U) {
        /* Backward-compat: treat 0 as "default max chunk". */
        req_len = ROBOT_FILE_CHUNK_SIZE;
    }
    if (req_len > ROBOT_FILE_CHUNK_SIZE) {
        APP_LOG_ERROR("Invalid chunk size: %u", (unsigned int)req->length);
        return app_file_send_error(seq, ROBOT_FILE_ERR_INVALID_REQ, req->filename);
    }

    /* Ensure null-terminated filename */
    char filename[ROBOT_FILE_MAX_FILENAME + 1];
    memcpy(filename, req->filename, ROBOT_FILE_MAX_FILENAME);
    filename[ROBOT_FILE_MAX_FILENAME] = '\0';

    /* Open file */
    FIL *file = &s_file_handle;
    memset(file, 0, sizeof(*file));
    FRESULT res = f_open(file, filename, FA_READ);
    if (res != FR_OK) {
        APP_LOG_ERROR("Failed to open '%s' (err=%d)", filename, res);
        return app_file_send_error(seq, ROBOT_FILE_ERR_NOT_FOUND, req->filename);
    }

    uint32_t total_size = f_size(file);

    /* Validate offset */
    if (req->offset >= total_size) {
        f_close(file);
        APP_LOG_ERROR("Invalid offset %lu >= %lu", (unsigned long)req->offset, (unsigned long)total_size);
        return app_file_send_error(seq, ROBOT_FILE_ERR_INVALID_REQ, req->filename);
    }

    /* Seek to offset */
    if (req->offset > 0U) {
        res = f_lseek(file, req->offset);
        if (res != FR_OK) {
            f_close(file);
            APP_LOG_ERROR("Seek failed (err=%d)", res);
            return app_file_send_error(seq, ROBOT_FILE_ERR_READ_ERROR, req->filename);
        }
    }

    /* Build response */
    uint8_t *resp_buf = s_file_resp_buf;
    const uint16_t resp_cap = (uint16_t)sizeof(s_file_resp_buf);
    robot_file_read_resp_t *resp = (robot_file_read_resp_t *)resp_buf;
    resp->offset = req->offset;
    resp->total_size = total_size;

    /* Read chunk */
    UINT bytes_read = 0;
    uint16_t chunk_max = req_len;
    /* Ensure response fits in frame */
    if (sizeof(robot_file_read_resp_t) + chunk_max > resp_cap) {
        chunk_max = resp_cap - sizeof(robot_file_read_resp_t);
    }

    res = f_read(file, resp_buf + sizeof(robot_file_read_resp_t), chunk_max, &bytes_read);
    f_close(file);

    if (res != FR_OK) {
        APP_LOG_ERROR("Read failed (err=%d)", res);
        return app_file_send_error(seq, ROBOT_FILE_ERR_READ_ERROR, req->filename);
    }

    resp->chunk_len = (uint16_t)bytes_read;
    uint16_t total_len = sizeof(robot_file_read_resp_t) + resp->chunk_len;

    if (req->offset == 0U) {
        APP_LOG_INFO("FILE_READ_RESP '%s' size=%lu chunk=%u resp_len=%u hdr_len=%u",
                     filename, (unsigned long)total_size, (unsigned int)bytes_read,
                     (unsigned int)total_len,
                     (unsigned int)sizeof(robot_file_read_resp_t));
    }

    bool ok = app_link_send(ROBOT_MSG_FILE_READ_RESP, ROBOT_FLAG_ACK_REQ,
                            resp_buf, total_len, seq);
    if (!ok && req->offset == 0U) {
        app_link_send_err_t err;
        uint32_t status = 0U;
        uint32_t hal_state = 0U;
        uint32_t hal_err = 0U;
        app_link_get_last_send_error(&err, &status, &hal_state, &hal_err);
        APP_LOG_ERROR("FILE_READ_RESP send failed (%s, status=%lu state=%lu err=0x%lx)",
                      app_link_send_err_str(err), (unsigned long)status,
                      (unsigned long)hal_state, (unsigned long)hal_err);
    }

    return ok;
}

void app_file_init(void)
{
    s_file_ctx.busy = false;
    if (retSD != 0U) {
        APP_LOG_ERROR("SD driver link failed (err=%u)", (unsigned int)retSD);
    } else {
        memset(&SDFatFS, 0, sizeof(SDFatFS));
        FRESULT res = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1);
        if (res != FR_OK) {
            APP_LOG_ERROR("SD mount failed (err=%d)", res);
            retSD = (uint8_t)res;
        } else {
            APP_LOG_INFO("SD mount OK");
            retSD = 0U;
        }
    }

    APP_LOG_INFO("File transfer initialized");
}

bool app_file_handle_message(uint8_t msg_type, uint16_t seq,
                              const uint8_t *payload, uint16_t payload_len)
{
    switch (msg_type) {
    case ROBOT_MSG_FILE_LIST_REQ:
        return app_file_handle_list(seq, payload, payload_len);

    case ROBOT_MSG_FILE_READ_REQ:
        return app_file_handle_read(seq, payload, payload_len);

    default:
        return false;
    }
}
