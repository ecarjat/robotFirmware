#include <catch2/catch_test_macros.hpp>

#include <cstring>
#include <limits>

extern "C" {
#include "crc32.h"
#include "logging/blackbox.h"
#include "qspi_fake.h"
#include "qspi_w25q64.h"
#include "stm32h7xx_hal.h"

void log_test_reset(void);
}

namespace {

robot_params_t test_params()
{
    robot_params_t params = {};
    params.control_rate_hz = 400.0f;
    return params;
}

void run_writer_ticks(unsigned int count)
{
    for (unsigned int i = 0; i < count; ++i) {
        log_writer_tick();
    }
}

LogMeta read_meta(uint32_t address)
{
    LogMeta meta = {};
    std::memcpy(&meta, qspi_fake_data() + address, sizeof(meta));
    return meta;
}

bool meta_is_valid(const LogMeta &meta)
{
    return std::memcmp(meta.magic, LOG_META_MAGIC, 7U) == 0 &&
           meta.version == LOG_META_VERSION &&
           meta.record_size == LOG_RECORD_SIZE &&
           meta.ring_start == LOG_RING_START &&
           meta.ring_size == LOG_RING_SIZE &&
           meta.write_addr >= LOG_RING_START &&
           meta.write_addr < LOG_RING_END &&
           ((meta.write_addr - LOG_RING_START) % LOG_RECORD_SIZE) == 0U &&
           meta.meta_crc32 ==
               robot_crc32(reinterpret_cast<const uint8_t *>(&meta),
                           sizeof(LogMeta) - sizeof(uint32_t));
}

void reset_logger_and_flash()
{
    qspi_fake_reset();
    log_test_reset();
    g_hal_tick = 0U;
    g_hal_delay_calls = 0U;
}

void initialize_two_verified_slots()
{
    const robot_params_t params = test_params();
    log_init(&params);
    run_writer_ticks(32U);

    g_hal_tick = LOG_META_UPDATE_PERIOD_MS;
    run_writer_ticks(32U);

    REQUIRE(meta_is_valid(read_meta(LOG_META_SLOT0)));
    REQUIRE(meta_is_valid(read_meta(LOG_META_SLOT1)));
    REQUIRE(read_meta(LOG_META_SLOT0).sequence == 0U);
    REQUIRE(read_meta(LOG_META_SLOT1).sequence == 1U);
}

void schedule_next_metadata_save()
{
    g_hal_tick += LOG_META_UPDATE_PERIOD_MS;
    log_writer_tick();
}

void write_raw_meta(uint32_t address, uint32_t sequence)
{
    LogMeta meta = {};
    std::memcpy(meta.magic, LOG_META_MAGIC, sizeof(meta.magic));
    meta.version = LOG_META_VERSION;
    meta.record_size = LOG_RECORD_SIZE;
    meta.rate_hz = 400U;
    meta.log_fields_mask = LOG_FIELDS_MASK_DEFAULT;
    meta.ring_start = LOG_RING_START;
    meta.ring_size = LOG_RING_SIZE;
    meta.write_addr = LOG_RING_START;
    meta.sequence = sequence;
    meta.meta_crc32 = robot_crc32(reinterpret_cast<const uint8_t *>(&meta),
                                  sizeof(meta) - sizeof(meta.meta_crc32));

    REQUIRE(qspi_w25q64_write_async_start(
        address, reinterpret_cast<const uint8_t *>(&meta), sizeof(meta)));
    REQUIRE(qspi_w25q64_write_async_tick() == QSPI_W25Q64_ASYNC_DONE);
}

void reboot_logger()
{
    const robot_params_t params = test_params();
    qspi_fake_power_cycle();
    log_test_reset();
    log_init(&params);
}

} // namespace

TEST_CASE("blackbox metadata keeps a verified copy in a separate erase sector",
          "[blackbox][metadata]")
{
    SECTION("normal alternating saves preserve the previous sector")
    {
        reset_logger_and_flash();
        initialize_two_verified_slots();
        const LogMeta previous = read_meta(LOG_META_SLOT1);

        schedule_next_metadata_save();
        qspi_fake_reset_counters();
        run_writer_ticks(8U);

        CHECK(qspi_fake_last_erase_addr() == LOG_META_SLOT0);
        CHECK(qspi_fake_last_write_addr() == LOG_META_SLOT0);
        CHECK(std::memcmp(&previous, qspi_fake_data() + LOG_META_SLOT1,
                          sizeof(previous)) == 0);
        CHECK(meta_is_valid(read_meta(LOG_META_SLOT0)));
        CHECK(meta_is_valid(read_meta(LOG_META_SLOT1)));
        CHECK(read_meta(LOG_META_SLOT0).sequence == 2U);
    }

    SECTION("power loss during target-sector erase retains the other copy")
    {
        reset_logger_and_flash();
        initialize_two_verified_slots();

        schedule_next_metadata_save();
        qspi_fake_reset_counters();
        qspi_fake_power_loss_during_next_erase();
        log_writer_tick();

        REQUIRE(qspi_fake_last_erase_addr() == LOG_META_SLOT0);
        reboot_logger();

        CHECK(log_get_write_addr() == LOG_RING_START);
        CHECK(meta_is_valid(read_meta(LOG_META_SLOT1)));
        CHECK(read_meta(LOG_META_SLOT1).sequence == 1U);
    }

    SECTION("power loss during target-sector program retains the other copy")
    {
        reset_logger_and_flash();
        initialize_two_verified_slots();

        schedule_next_metadata_save();
        log_writer_tick(); // Start slot 0 erase.
        log_writer_tick(); // Complete erase and move to program state.
        qspi_fake_reset_counters();
        qspi_fake_power_loss_during_next_write(sizeof(LogMeta) / 2U);
        log_writer_tick();

        REQUIRE(qspi_fake_last_write_addr() == LOG_META_SLOT0);
        reboot_logger();

        CHECK(log_get_write_addr() == LOG_RING_START);
        CHECK_FALSE(meta_is_valid(read_meta(LOG_META_SLOT0)));
        CHECK(meta_is_valid(read_meta(LOG_META_SLOT1)));
        CHECK(read_meta(LOG_META_SLOT1).sequence == 1U);
    }

    SECTION("a failed readback retries the same inactive sector")
    {
        reset_logger_and_flash();
        initialize_two_verified_slots();

        schedule_next_metadata_save();
        log_writer_tick(); // Start slot 0 erase.
        log_writer_tick(); // Complete erase and move to program state.
        log_writer_tick(); // Start slot 0 program.
        log_writer_tick(); // Complete program and enter readback verification.
        qspi_fake_fail_next_read();
        log_writer_tick(); // Readback fails; sequence must remain uncommitted.

        g_hal_tick += LOG_META_UPDATE_PERIOD_MS;
        qspi_fake_reset_counters();
        log_writer_tick(); // Queue the retry.
        run_writer_ticks(8U);

        CHECK(qspi_fake_last_erase_addr() == LOG_META_SLOT0);
        CHECK(meta_is_valid(read_meta(LOG_META_SLOT1)));
        CHECK(read_meta(LOG_META_SLOT1).sequence == 1U);
        CHECK(meta_is_valid(read_meta(LOG_META_SLOT0)));
        CHECK(read_meta(LOG_META_SLOT0).sequence == 2U);
    }

    SECTION("sequence rollover chooses zero after UINT32_MAX")
    {
        reset_logger_and_flash();
        write_raw_meta(LOG_META_SLOT0, std::numeric_limits<uint32_t>::max() - 1U);
        write_raw_meta(LOG_META_SLOT1, std::numeric_limits<uint32_t>::max());

        const robot_params_t params = test_params();
        log_init(&params);
        qspi_fake_reset_counters();
        log_writer_tick();

        CHECK(qspi_fake_last_erase_addr() == LOG_META_SLOT0);
    }
}
