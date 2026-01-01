# SPI Sensor Scheduler

This document specifies the shared SPI sensor scheduler used to arbitrate the
bus between multiple IMUs (ICM42688, BMI270, and future sensors).

## Goals
- Prevent starvation when multiple sensors share one SPI bus.
- Keep latency low by starting the next DMA transfer immediately after the
  previous one completes.
- Minimize CPU usage (interrupt handlers are O(1) and do no bus work).
- Deliver latest-only samples for the state estimator.

## Ownership Model
- The scheduler is the only runtime owner of the SPI bus.
- Sensor drivers may use blocking register reads/writes during init while the
  bus is not "ready".
- After init, all runtime reads are driven by the scheduler via DMA.

## Interrupt Handling
- EXTI handlers do not start DMA.
- Each sensor ISR only sets a "pending" flag (and optional timestamp).
- Pending flags collapse multiple interrupts into one request (latest-only).

## Scheduler Algorithm
Data structures (conceptual):
- `pending_mask`: bit per sensor.
- `rr_cursor`: round-robin index.
- Optional `pending_ts[]`: last IRQ time to break ties by age.
- Optional `min_interval_ms[]`: per-sensor minimum service interval.
- Optional `last_served_ms[]`: per-sensor last service time.

Selection rule:
1) If no pending bits, return.
2) Build an "eligible" mask: pending sensors whose `now - last_served_ms`
   is >= `min_interval_ms` (or interval == 0).
3) If eligible mask is empty, fall back to any pending sensor (oldest-first)
   to avoid starvation.
4) Pick next sensor in round-robin order within the eligible set.
5) Clear its pending bit and start DMA via the sensor's `kick()` function.
6) On DMA completion, update `last_served_ms[sensor]` and the bus idle hook
   calls the scheduler again.
7) If DMA fails or bus is busy, re-set the pending bit.

This guarantees fairness (no starvation) while keeping latency low because the
next transfer is scheduled immediately after DMA completion.

## Per-Sensor Rate Control (Priorities Without Starvation)
- Each sensor can be assigned a `min_interval_ms` (0 = no limit).
- Configure defaults in `Drivers/imu/imu_sched_config.h` or set at runtime via
  `imu_sched_set_min_interval()`.
- A smaller interval means higher priority (more frequent service).
- The eligible-mask rule keeps high-rate sensors fast, but the fallback
  "any pending" step prevents starvation of low-rate sensors if high-rate
  sensors are constantly pending.

Example:
- ICM42688: 2 ms
- BMI270: 5 ms
- Future magnetometer: 20 ms

The scheduler will prefer ICM when it is eligible, but still guarantees BMI and
magnetometer get serviced even if ICM keeps firing.

## Bus Hooks
- `spi_bus_idle_hook()` calls `imu_sched_run()` to schedule the next transfer.
- Optionally call `imu_sched_run()` from a low-rate timer to recover from
  missed interrupts or DMA errors.

## Latest-Only Semantics
- While pending, additional interrupts do not queue; they only keep the pending
  bit set.
- The scheduler always fetches the newest data available at the next bus turn.

## Removal of imu_*_poll()
To avoid interfering with the scheduler:
- `imu_*_poll()` is removed from the runtime path and is not called from the
  main loop.
- All DMA starts are initiated only by the scheduler.
- Any diagnostics/timeouts should move to scheduler-side checks or a separate
  `imu_*_diag()` function that does not touch the bus.

## Integration Steps
1) Add a scheduler module (e.g., `imu_sched.c/.h`).
2) EXTI callbacks set `pending` bits for the corresponding sensor.
3) Scheduler owns all runtime DMA kicks.
4) App loop stops calling `imu_*_poll()`; optional diagnostics are separate.
