#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstring>

extern "C" {
#include "fatfs_fake.h"
#include "logging/blackbox.h"
#include "logging/blackbox_dump.h"
#include "qspi_fake.h"
#include "stm32h7xx_hal.h"
}

namespace {

uint32_t s_next_sequence = 0U;

void push_record()
{
    alignas(LogRecord) std::array<uint8_t, LOG_RECORD_SIZE> storage = {};
    auto *record = reinterpret_cast<LogRecord *>(storage.data());
    record->magic = LOG_RECORD_MAGIC;
    record->version = LOG_RECORD_VERSION;
    record->seq = s_next_sequence++;
    log_push_record(record);
}

void idle_tick(bool export_allowed)
{
    log_writer_tick();
    log_erase_tick();
    log_dump_tick(export_allowed);
}

void wait_for_capture_while_balancing()
{
    for (int i = 0; i < 256; ++i) {
        idle_tick(false);
        if (log_capture_poll(nullptr) == LOG_CAPTURE_READY) {
            /* Advance CAPTURE_FLUSHING to CAPTURED_WAIT_SAFE without export. */
            log_dump_tick(false);
            return;
        }
    }
    FAIL("capture did not commit through its watermark");
}

void finish_export()
{
    for (int i = 0; i < 256 && log_is_dumping(); ++i) {
        idle_tick(true);
    }
    CHECK_FALSE(log_is_dumping());
}

void expect_failure_then_recover(fatfs_fake_failure_t failure)
{
    push_record();
    REQUIRE(log_dump_last_seconds(1U));
    wait_for_capture_while_balancing();

    fatfs_fake_reset();
    fatfs_fake_set_failure(failure);
    finish_export();

    if (failure == FATFS_FAKE_FAIL_OPEN) {
        CHECK(fatfs_fake_close_calls() == 0U);
        CHECK(fatfs_fake_unlink_calls() == 0U);
    } else {
        CHECK(fatfs_fake_close_calls() >= 1U);
        CHECK(fatfs_fake_unlink_calls() == 1U);
    }

    /* Failures release the capture so a new request can be accepted. */
    push_record();
    REQUIRE(log_dump_last_seconds(1U));
    wait_for_capture_while_balancing();
    fatfs_fake_reset();
    finish_export();
}

} // namespace

TEST_CASE("deferred blackbox export preserves a fixed wrapped snapshot",
          "[blackbox][dump]")
{
    qspi_fake_reset();
    fatfs_fake_reset();
    s_next_sequence = 0U;
    g_hal_tick = 0U;
    g_hal_delay_calls = 0U;

    robot_params_t params = {};
    params.control_rate_hz = 1.0f;
    log_init(&params);
    log_dump_init();

    /* Drive the real logger beyond the QSPI ring once. The writer flushes
     * in 4 KB chunks, so a later 30-record dump must read across ring wrap. */
    bool wrapped = false;
    for (uint32_t i = 0; i < 60000U && !wrapped; ++i) {
        push_record();
        log_writer_tick();
        log_writer_tick();

        log_stats_t stats = {};
        log_get_stats(&stats);
        wrapped = stats.wrap_count > 0U;
    }
    REQUIRE(wrapped);

    /* Ensure the capture endpoint includes records that arrived immediately
     * before the request, but excludes everything that arrives after it. */
    push_record();
    push_record();
    push_record();
    const uint32_t captured_end_sequence = s_next_sequence;

    qspi_fake_reset_counters();
    g_hal_delay_calls = 0U;
    REQUIRE(log_dump_last_seconds(30U));
    CHECK_FALSE(log_dump_last_seconds(30U));
    CHECK(g_hal_delay_calls == 0U);
    CHECK(qspi_fake_read_calls() == 0U);
    CHECK(fatfs_fake_opendir_calls() == 0U);
    CHECK(fatfs_fake_open_calls() == 0U);

    /* This post-trigger record must not be part of the dump. */
    push_record();
    wait_for_capture_while_balancing();

    CHECK(qspi_fake_read_calls() == 0U);
    CHECK(fatfs_fake_opendir_calls() == 0U);
    CHECK(fatfs_fake_open_calls() == 0U);
    CHECK(fatfs_fake_write_calls() == 0U);
    CHECK(g_hal_delay_calls == 0U);

    finish_export();

    REQUIRE(qspi_fake_read_calls() == 2U);
    CHECK(fatfs_fake_opendir_calls() == 1U);
    CHECK(fatfs_fake_open_calls() == 1U);

    const uint8_t *file = fatfs_fake_file_data();
    const size_t expected_size = sizeof(LogMeta) + 30U * LOG_RECORD_SIZE +
                                 sizeof(LogDumpTrailer);
    REQUIRE(fatfs_fake_file_size() == expected_size);

    LogMeta metadata = {};
    std::memcpy(&metadata, file, sizeof(metadata));
    CHECK(metadata.write_addr == log_get_write_addr());

    for (uint32_t i = 0; i < 30U; ++i) {
        LogRecord record = {};
        const size_t offset = sizeof(LogMeta) + i * LOG_RECORD_SIZE;
        std::memcpy(&record, file + offset, sizeof(record));
        CHECK(record.seq == captured_end_sequence - 30U + i);
    }

    LogDumpTrailer trailer = {};
    std::memcpy(&trailer, file + sizeof(LogMeta) + 30U * LOG_RECORD_SIZE,
                sizeof(trailer));
    CHECK(trailer.start_seq == captured_end_sequence - 30U);
    CHECK(trailer.end_seq == captured_end_sequence);
    CHECK(trailer.record_count == 30U);

    /* Every filesystem failure closes/unlinks best-effort, releases capture,
     * and allows a later independent request. */
    expect_failure_then_recover(FATFS_FAKE_FAIL_OPEN);
    expect_failure_then_recover(FATFS_FAKE_FAIL_WRITE);
    expect_failure_then_recover(FATFS_FAKE_FAIL_SYNC);
    expect_failure_then_recover(FATFS_FAKE_FAIL_CLOSE);

    /* The logger's retention guard rejects new records once the protected
     * range would otherwise be overwritten, then resumes after release. */
    push_record();
    REQUIRE(log_capture_begin());
    for (int i = 0; i < 256; ++i) {
        log_writer_tick();
        if (log_capture_poll(nullptr) == LOG_CAPTURE_READY) {
            break;
        }
    }
    REQUIRE(log_capture_poll(nullptr) == LOG_CAPTURE_READY);
    REQUIRE(log_capture_protect(LOG_RING_SIZE / LOG_RECORD_SIZE));
    log_writer_tick();

    log_stats_t before_hold = {};
    log_get_stats(&before_hold);
    push_record();
    log_stats_t held = {};
    log_get_stats(&held);
    CHECK(held.dropped_records == before_hold.dropped_records + 1U);

    log_capture_release();
    push_record();
    log_stats_t released = {};
    log_get_stats(&released);
    CHECK(released.dropped_records == held.dropped_records);
}
