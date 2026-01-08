#include "app_link.h"
#include "app_cmd.h"
#include "app_config.h"
#include "app_rpc.h"
#include "app_utils.h"
#include "crc32.h"
#include "framing_cobs.h"
#include "led_status.h"
#include "mux_channels.h"

static uint8_t s_uart_rx_buffer[APP_LINK_RX_BUFFER_BYTES]
    __attribute__((section(".dma_buffer"), aligned(32)));
static uint8_t s_link_tx_queue[APP_LINK_TX_QUEUE_DEPTH][ROBOT_FRAME_MAX_ENCODED]
    __attribute__((section(".dma_buffer"), aligned(32)));
static uint16_t s_link_tx_len[APP_LINK_TX_QUEUE_DEPTH];

/* CDC Ring Buffer */
#define APP_LINK_CDC_BUFFER_BYTES 1024U
static uint8_t s_cdc_rx_buffer[APP_LINK_CDC_BUFFER_BYTES];
static volatile size_t s_cdc_rx_head = 0U;
static volatile size_t s_cdc_rx_tail = 0U;

static robot_mux_t s_mux;

static volatile uint8_t s_link_tx_busy = 0U;
static volatile uint8_t s_link_tx_head = 0U;
static volatile uint8_t s_link_tx_tail = 0U;

static uint16_t s_seq_counters[ROBOT_CHANNEL_MAX + 1U] = {0};
static size_t s_uart_rx_last_pos = 0U;
static volatile uint16_t s_uart_rx_pending_size = 0U;
static volatile uint8_t s_uart_rx_event_pending = 0U;

typedef struct {
  uint8_t buffer[APP_LINK_FRAME_BUFFER_BYTES];
  size_t len;
} link_decoder_t;

static link_decoder_t s_uart_decoder;
static link_decoder_t s_cdc_decoder;

static volatile uint32_t s_link_overflows = 0U;
static volatile uint32_t s_link_decode_failures = 0U;
static volatile uint32_t s_link_decode_last_len = 0U;
static volatile uint32_t s_link_uart_errors = 0U;
static volatile uint32_t s_link_uart_last_err = 0U;
static volatile app_link_send_err_t s_link_send_last_err = APP_LINK_SEND_OK;
static volatile uint32_t s_link_send_last_status = 0U;
static volatile uint32_t s_link_send_last_hal_state = 0U;
static volatile uint32_t s_link_send_last_hal_err = 0U;
uint32_t s_last_cmd_ms = 0U;

static void app_link_flush_frame(link_decoder_t *ctx);
static void app_link_handle_encoded_frame(const uint8_t *frame, size_t len);
static void app_link_log_bytes(const char *label, const uint8_t *data,
                               size_t len);
static void app_link_restart_rx(void);

static bool app_link_enqueue_encoded(const uint8_t *encoded,
                                     uint16_t encoded_len);
static void app_link_clear_uart_errors(UART_HandleTypeDef *huart);
static void app_link_invalidate_rx_cache(size_t len);
static bool app_link_dma_is_circular(void);
static void app_link_kick_tx(void);
static void app_link_process_chunk_ctx(link_decoder_t *ctx, const uint8_t *data,
                                       size_t len);

void app_link_get_last_send_error(app_link_send_err_t *err, uint32_t *status,
                                  uint32_t *hal_state, uint32_t *hal_err) {
  if (err)
    *err = s_link_send_last_err;
  if (status)
    *status = s_link_send_last_status;
  if (hal_state)
    *hal_state = s_link_send_last_hal_state;
  if (hal_err)
    *hal_err = s_link_send_last_hal_err;
}

static void app_link_clear_uart_errors(UART_HandleTypeDef *huart) {
  if (huart == NULL) {
    return;
  }

  __HAL_UART_CLEAR_PEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_IDLEFLAG(huart);
  __HAL_UART_FLUSH_DRREGISTER(huart);
}

const char *app_link_send_err_str(app_link_send_err_t err) {
  switch (err) {
  case APP_LINK_SEND_OK:
    return "ok";
  case APP_LINK_SEND_ERR_UART_NULL:
    return "uart_null";
  case APP_LINK_SEND_ERR_ISR:
    return "in_isr";
  case APP_LINK_SEND_ERR_FRAME_INIT:
    return "frame_init";
  case APP_LINK_SEND_ERR_ENCODE:
    return "encode";
  case APP_LINK_SEND_ERR_CTS_BLOCKED:
    return "cts_blocked";
  case APP_LINK_SEND_ERR_UART_BUSY:
    return "uart_busy";
  case APP_LINK_SEND_ERR_UART_TIMEOUT:
    return "uart_timeout";
  case APP_LINK_SEND_ERR_UART_ERROR:
    return "uart_error";
  default:
    return "unknown";
  }
}

static void app_link_invalidate_rx_cache(size_t len) {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  if (len == 0U) {
    return;
  }
  uintptr_t start = (uintptr_t)s_uart_rx_buffer;
  uintptr_t end = start + len;
  uintptr_t aligned_start = start & ~(uintptr_t)(32U - 1U);
  uintptr_t aligned_end = (end + (32U - 1U)) & ~(uintptr_t)(32U - 1U);
  SCB_InvalidateDCache_by_Addr((uint32_t *)aligned_start,
                               (int32_t)(aligned_end - aligned_start));
#else
  (void)len;
#endif
}

static void app_link_clean_tx_cache(const uint8_t *data, size_t len) {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  if (data == NULL || len == 0U) {
    return;
  }
  uintptr_t start = (uintptr_t)data;
  uintptr_t end = start + len;
  uintptr_t aligned_start = start & ~(uintptr_t)(32U - 1U);
  uintptr_t aligned_end = (end + (32U - 1U)) & ~(uintptr_t)(32U - 1U);
  SCB_CleanDCache_by_Addr((uint32_t *)aligned_start,
                          (int32_t)(aligned_end - aligned_start));
#else
  (void)data;
  (void)len;
#endif
}

static bool app_link_dma_is_circular(void) {
  if (APP_LINK_UART == NULL || APP_LINK_UART->hdmarx == NULL) {
    return false;
  }
  return (APP_LINK_UART->hdmarx->Init.Mode == DMA_CIRCULAR);
}

static void app_link_kick_tx(void) {
  if (APP_LINK_UART == NULL) {
    return;
  }
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (s_link_tx_busy || s_link_tx_tail == s_link_tx_head) {
    __set_PRIMASK(primask);
    return;
  }
  uint8_t idx = s_link_tx_tail;
  s_link_tx_busy = 1U;
  __set_PRIMASK(primask);

  app_link_clean_tx_cache(s_link_tx_queue[idx], s_link_tx_len[idx]);
  HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(
      APP_LINK_UART, s_link_tx_queue[idx], s_link_tx_len[idx]);
  if (status == HAL_BUSY) {
    /* UART still busy: clear busy flag and keep the frame queued for a later
     * kick */
    primask = __get_PRIMASK();
    __disable_irq();
    s_link_tx_busy = 0U;
    __set_PRIMASK(primask);
  } else if (status != HAL_OK) {
    s_link_send_last_status = (uint32_t)status;
    s_link_send_last_hal_state = (uint32_t)APP_LINK_UART->gState;
    s_link_send_last_hal_err = (uint32_t)APP_LINK_UART->ErrorCode;
    if (status == HAL_BUSY) {
      s_link_send_last_err = APP_LINK_SEND_ERR_UART_BUSY;
    } else if (status == HAL_TIMEOUT) {
      s_link_send_last_err = APP_LINK_SEND_ERR_UART_TIMEOUT;
    } else {
      s_link_send_last_err = APP_LINK_SEND_ERR_UART_ERROR;
    }
    /* Drop this frame to avoid deadlock and clear busy. */
    primask = __get_PRIMASK();
    __disable_irq();
    s_link_tx_busy = 0U;
    s_link_tx_tail = (uint8_t)((s_link_tx_tail + 1U) % APP_LINK_TX_QUEUE_DEPTH);
    __set_PRIMASK(primask);
    app_link_kick_tx();
  }
}

static bool app_link_enqueue_encoded(const uint8_t *encoded,
                                     uint16_t encoded_len) {
  if (encoded == NULL || encoded_len == 0U ||
      encoded_len > ROBOT_FRAME_MAX_ENCODED) {
    return false;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint8_t next_head =
      (uint8_t)((s_link_tx_head + 1U) % APP_LINK_TX_QUEUE_DEPTH);
  if (next_head == s_link_tx_tail) {
    __set_PRIMASK(primask);
    return false; /* queue full */
  }
  memcpy(s_link_tx_queue[s_link_tx_head], encoded, encoded_len);
  s_link_tx_len[s_link_tx_head] = encoded_len;
  s_link_tx_head = next_head;
  __set_PRIMASK(primask);

  app_link_kick_tx();
  return true;
}

bool app_link_send(uint8_t type, uint16_t flags, const uint8_t *payload,
                   uint16_t len, uint16_t seq_override) {
  robot_frame_t frame;
  uint8_t encoded[ROBOT_FRAME_MAX_ENCODED];
  size_t encoded_len = 0U;

  s_link_send_last_err = APP_LINK_SEND_OK;
  s_link_send_last_status = 0U;
  s_link_send_last_hal_state = 0U;
  s_link_send_last_hal_err = 0U;

  if (APP_LINK_UART == NULL) {
    s_link_send_last_err = APP_LINK_SEND_ERR_UART_NULL;
    return false;
  }
  if (app_in_isr()) {
    s_link_send_last_err = APP_LINK_SEND_ERR_ISR;
    return false;
  }

  uint16_t seq = (seq_override != 0U)
                     ? seq_override
                     : ++s_seq_counters[robot_channel_from_type(type)];
  if (!robot_frame_init(&frame, type, seq, flags, payload, len)) {
    s_link_send_last_err = APP_LINK_SEND_ERR_FRAME_INIT;
    return false;
  }
  if (!robot_frame_encode(&frame, encoded, sizeof(encoded), &encoded_len)) {
    s_link_send_last_err = APP_LINK_SEND_ERR_ENCODE;
    return false;
  }

  if (!app_link_enqueue_encoded(encoded, (uint16_t)encoded_len)) {
    s_link_send_last_err = APP_LINK_SEND_ERR_UART_BUSY;
    return false; /* drop: queue full */
  }

  return true;
}

void app_link_start(void) {
  if (APP_LINK_UART == NULL) {
    APP_LOG_ERROR("Link UART not initialized");
    return;
  }
  robot_mux_init(&s_mux);
  robot_mux_register(&s_mux, ROBOT_CHANNEL_CMD, app_cmd_handler, NULL);

  app_link_clear_uart_errors(APP_LINK_UART);
  if (HAL_UARTEx_ReceiveToIdle_DMA(APP_LINK_UART, s_uart_rx_buffer,
                                   sizeof(s_uart_rx_buffer)) != HAL_OK) {
    APP_LOG_ERROR("UART RX start failed");
    return;
  }
  __HAL_DMA_DISABLE_IT(APP_LINK_UART->hdmarx, DMA_IT_HT);
  s_uart_rx_last_pos = 0U;
  s_uart_rx_event_pending = 0U;
  s_uart_rx_pending_size = 0U;

  /* Reset decoders */
  s_uart_decoder.len = 0U;
  s_cdc_decoder.len = 0U;
}

bool app_link_feed_cdc(const uint8_t *data, uint32_t len) {
  if (data == NULL || len == 0U) {
    return false;
  }
  /* Simple ring buffer push (ISR safe for single consumer) */
  for (uint32_t i = 0; i < len; i++) {
    size_t next = (s_cdc_rx_head + 1) % APP_LINK_CDC_BUFFER_BYTES;
    if (next == s_cdc_rx_tail) {
      s_link_overflows++;
      return false; /* Buffer full */
    }
    s_cdc_rx_buffer[s_cdc_rx_head] = data[i];
    s_cdc_rx_head = next;
  }
  return true;
}

void app_link_poll(void) {
  /* 1. Process UART RX */
  if (APP_LINK_UART == NULL) {
    goto process_cdc;
  }

  if (app_link_dma_is_circular()) {
    size_t buf_size = sizeof(s_uart_rx_buffer);
    uint16_t remaining = __HAL_DMA_GET_COUNTER(APP_LINK_UART->hdmarx);
    size_t pos = buf_size - (size_t)remaining;
    if (pos >= buf_size) {
      pos = 0U;
    }
    if (pos == s_uart_rx_last_pos) {
      goto process_cdc;
    }

    app_link_invalidate_rx_cache(sizeof(s_uart_rx_buffer));
    if (pos > s_uart_rx_last_pos) {
      app_link_process_chunk_ctx(&s_uart_decoder,
                                 &s_uart_rx_buffer[s_uart_rx_last_pos],
                                 pos - s_uart_rx_last_pos);
    } else {
      app_link_process_chunk_ctx(&s_uart_decoder,
                                 &s_uart_rx_buffer[s_uart_rx_last_pos],
                                 buf_size - s_uart_rx_last_pos);
      if (pos > 0U) {
        app_link_process_chunk_ctx(&s_uart_decoder, &s_uart_rx_buffer[0], pos);
      }
    }
    s_uart_rx_last_pos = pos;

  } else if (s_uart_rx_event_pending) {
    /* Idle line detection mode */
    __disable_irq();
    uint16_t size = s_uart_rx_pending_size;
    s_uart_rx_pending_size = 0U;
    s_uart_rx_event_pending = 0U;
    __enable_irq();

    if (size == 0U) {
      goto process_cdc;
    }

    app_link_invalidate_rx_cache(size);
    app_link_process_chunk_ctx(&s_uart_decoder, s_uart_rx_buffer, size);
    app_link_restart_rx();
  }

process_cdc:
  /* 2. Process CDC RX */
  if (s_cdc_rx_head != s_cdc_rx_tail) {
    /* Process in chunks to avoid holding up the loop too long */
    uint32_t count = 0;
    uint8_t chunk[64];
    while (s_cdc_rx_head != s_cdc_rx_tail && count < sizeof(chunk)) {
      chunk[count++] = s_cdc_rx_buffer[s_cdc_rx_tail];
      s_cdc_rx_tail = (s_cdc_rx_tail + 1) % APP_LINK_CDC_BUFFER_BYTES;
    }
    if (count > 0) {
      app_link_process_chunk_ctx(&s_cdc_decoder, chunk, count);
    }
  }
}

static void app_link_restart_rx(void) {
  if (app_link_dma_is_circular()) {
    return;
  }
  app_link_clear_uart_errors(APP_LINK_UART);
  if (HAL_UARTEx_ReceiveToIdle_DMA(APP_LINK_UART, s_uart_rx_buffer,
                                   sizeof(s_uart_rx_buffer)) != HAL_OK) {
    APP_LOG_ERROR("UART RX restart failed");
    return;
  }
  __HAL_DMA_DISABLE_IT(APP_LINK_UART->hdmarx, DMA_IT_HT);
  s_uart_rx_last_pos = 0U;
}

static void app_link_process_chunk_ctx(link_decoder_t *ctx, const uint8_t *data,
                                       size_t len) {
  for (size_t i = 0; i < len; ++i) {
    uint8_t byte = data[i];
    if (byte == 0x00U) {
      app_link_flush_frame(ctx);
    } else if (ctx->len < sizeof(ctx->buffer)) {
      ctx->buffer[ctx->len++] = byte;
    } else {
      if (app_in_isr()) {
        s_link_overflows++;
      } else {
        APP_LOG_ERROR("UART frame overflow, dropping data");
      }
      ctx->len = 0U;
    }
  }
}

static void app_link_flush_frame(link_decoder_t *ctx) {
  if (ctx->len == 0U) {
    return;
  }

  app_link_handle_encoded_frame(ctx->buffer, ctx->len);
  ctx->len = 0U;
}

static void app_link_handle_encoded_frame(const uint8_t *frame, size_t len) {
  robot_frame_t decoded;

  if (len > ROBOT_FRAME_MAX_ENCODED) {
    APP_LOG_ERROR("Encoded frame too long (%u)", (unsigned int)len);
    return;
  }

  uint8_t encoded_buf[ROBOT_FRAME_MAX_ENCODED + 1];
  memcpy(encoded_buf, frame, len);
  encoded_buf[len] = 0x00U;

  if (!robot_frame_decode(encoded_buf, len + 1U, &decoded)) {
    s_link_decode_failures++;
    s_link_decode_last_len = (uint32_t)len;
    if (!app_in_isr()) {
      APP_LOG_ERROR("Frame decode failed (len=%u)", (unsigned int)len);
      app_link_debug_frame(frame, len);
    }
    return;
  }

  /* Auto-ACK if requested */
  if ((decoded.hdr.flags & ROBOT_FLAG_ACK_REQ) != 0U) {
    robot_frame_t ack;
    uint8_t encoded_ack[ROBOT_FRAME_MAX_ENCODED];
    size_t encoded_ack_len = 0U;
    if (robot_frame_init(&ack, ROBOT_MSG_ACK, decoded.hdr.seq,
                         ROBOT_FLAG_IS_ACK, NULL, 0U) &&
        robot_frame_encode(&ack, encoded_ack, sizeof(encoded_ack),
                           &encoded_ack_len)) {
      (void)app_link_enqueue_encoded(encoded_ack, (uint16_t)encoded_ack_len);
    }
  }

  app_link_dispatch(&decoded);
}

void app_log_link_errors(void) {
  uint32_t decode_fail = 0U;
  uint32_t decode_len = 0U;
  uint32_t overflow = 0U;
  uint32_t uart_errs = 0U;
  uint32_t uart_last = 0U;

  __disable_irq();
  decode_fail = s_link_decode_failures;
  decode_len = s_link_decode_last_len;
  s_link_decode_failures = 0U;
  overflow = s_link_overflows;
  s_link_overflows = 0U;
  uart_errs = s_link_uart_errors;
  uart_last = s_link_uart_last_err;
  s_link_uart_errors = 0U;
  __enable_irq();

  if (decode_fail > 0U) {
    APP_LOG_ERROR("Frame decode failed x%lu (last len=%lu)",
                  (unsigned long)decode_fail, (unsigned long)decode_len);
  }
  if (overflow > 0U) {
    APP_LOG_ERROR("UART frame overflow x%lu", (unsigned long)overflow);
  }
  if (uart_errs > 0U) {
    APP_LOG_ERROR("UART error 0x%lx x%lu", (unsigned long)uart_last,
                  (unsigned long)uart_errs);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart != APP_LINK_UART) {
    return;
  }
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_link_tx_tail = (uint8_t)((s_link_tx_tail + 1U) % APP_LINK_TX_QUEUE_DEPTH);
  s_link_tx_busy = 0U;
  __set_PRIMASK(primask);
  app_link_kick_tx();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart != APP_LINK_UART) {
    return;
  }
  if (app_link_dma_is_circular()) {
    s_uart_rx_event_pending = 1U;
    return;
  }

  s_uart_rx_pending_size = Size;
  s_uart_rx_event_pending = 1U;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart != APP_LINK_UART) {
    return;
  }
  s_link_uart_errors++;
  s_link_uart_last_err = (uint32_t)huart->ErrorCode;
  if (app_link_dma_is_circular()) {
    (void)HAL_UART_AbortReceive(huart);
    app_link_clear_uart_errors(huart);
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, s_uart_rx_buffer,
                                     sizeof(s_uart_rx_buffer)) != HAL_OK) {
      APP_LOG_ERROR("UART RX restart failed");
      return;
    }
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    s_uart_rx_last_pos = 0U;
    return;
  }

  app_link_restart_rx();
}

static void app_link_log_bytes(const char *label, const uint8_t *data,
                               size_t len) {
  if (label == NULL) {
    return;
  }
  if (data == NULL || len == 0U) {
    APP_LOG_ERROR("%s: <empty>", label);
    return;
  }

  size_t max_len = len;
  if (max_len > APP_LINK_DEBUG_MAX_BYTES) {
    max_len = APP_LINK_DEBUG_MAX_BYTES;
  }

  char line[APP_LOG_BUFFER_BYTES];
  int written =
      snprintf(line, sizeof(line), "%s (%u):", label, (unsigned int)len);
  if (written < 0) {
    return;
  }

  for (size_t i = 0U; i < max_len; ++i) {
    if ((size_t)written >= sizeof(line)) {
      break;
    }
    int ret = snprintf(line + written, sizeof(line) - (size_t)written, " %02X",
                       data[i]);
    if (ret < 0) {
      break;
    }
    written += ret;
  }

  if (max_len < len && (size_t)written < sizeof(line)) {
    (void)snprintf(line + written, sizeof(line) - (size_t)written, " ...");
  }

  APP_LOG_ERROR("%s", line);
}

void app_link_debug_frame(const uint8_t *frame, size_t len) {
#if APP_LINK_DEBUG_FRAMES
  static uint32_t s_debug_reports = 0U;
  if (s_debug_reports >= APP_LINK_DEBUG_MAX_REPORTS) {
    if (s_debug_reports == APP_LINK_DEBUG_MAX_REPORTS) {
      APP_LOG_ERROR("Frame debug suppressed (max reports reached)");
    }
    ++s_debug_reports;
    return;
  }
  ++s_debug_reports;

  APP_LOG_ERROR("Frame debug: encoded_len=%u", (unsigned int)len);
  app_link_log_bytes("Encoded", frame, len);

  if (len == 0U) {
    APP_LOG_ERROR("Frame debug: empty encoded payload");
    return;
  }

  uint8_t decoded[ROBOT_FRAME_MAX_DECODED];
  size_t decoded_len = robot_cobs_decode(frame, len, decoded, sizeof(decoded));
  if (decoded_len == 0U) {
    APP_LOG_ERROR("Frame debug: COBS decode failed");
    return;
  }

  APP_LOG_ERROR("Frame debug: decoded_len=%u", (unsigned int)decoded_len);
  app_link_log_bytes("Decoded", decoded, decoded_len);

  if (decoded_len < sizeof(robot_frame_header_t) + sizeof(uint32_t)) {
    APP_LOG_ERROR("Frame debug: decoded too short for header+CRC");
    return;
  }

  robot_frame_header_t hdr;
  memcpy(&hdr, decoded, sizeof(hdr));
  APP_LOG_ERROR(
      "Frame debug: hdr magic=0x%04x ver=%u type=0x%02x seq=%u len=%u "
      "flags=0x%04x",
      hdr.magic, hdr.version, hdr.type, hdr.seq, hdr.len, hdr.flags);

  if (hdr.magic != ROBOT_FRAME_MAGIC) {
    APP_LOG_ERROR("Frame debug: header magic mismatch");
  }
  if (hdr.version != ROBOT_FRAME_VERSION) {
    APP_LOG_ERROR("Frame debug: header version mismatch");
  }
  if (hdr.len > ROBOT_FRAME_MAX_PAYLOAD) {
    APP_LOG_ERROR("Frame debug: header len too large");
    return;
  }

  size_t crc_offset = sizeof(hdr) + (size_t)hdr.len;
  size_t expected_len = crc_offset + sizeof(uint32_t);
  if (expected_len != decoded_len) {
    APP_LOG_ERROR("Frame debug: length mismatch expected=%u decoded=%u",
                  (unsigned int)expected_len, (unsigned int)decoded_len);
  }
  if (expected_len > decoded_len) {
    return;
  }

  uint32_t crc_rx = 0U;
  memcpy(&crc_rx, decoded + crc_offset, sizeof(crc_rx));
  uint32_t crc_calc = robot_crc32(decoded, crc_offset);
  if (crc_rx != crc_calc) {
    APP_LOG_ERROR("Frame debug: crc rx=0x%08lx calc=0x%08lx",
                  (unsigned long)crc_rx, (unsigned long)crc_calc);
  } else {
    APP_LOG_ERROR("Frame debug: crc ok 0x%08lx", (unsigned long)crc_rx);
  }
#else
  (void)frame;
  (void)len;
#endif
}

void app_link_dispatch(const robot_frame_t *frame) {
  if (frame == NULL) {
    return;
  }

  /* Any valid frame counts as link liveness. */
  s_last_cmd_ms = HAL_GetTick();
  led_status_clear_flag(LED_STATUS_TELEM_FAILURE);

  if (frame->hdr.type == ROBOT_MSG_RPC_REQ) {
    app_rpc_handle(frame);
    return;
  }

  robot_mux_dispatch(&s_mux, frame->hdr.type, frame->payload, frame->hdr.len);
}