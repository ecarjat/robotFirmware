# Motor Link Protocol Code Review

**Date:** January 3, 2026
**Files Reviewed:**
- `firmware/Drivers/motors/motor_link.c` (STM32H7 controller)
- `simpleFocDriver/src/comms_streams.cpp` (STM32F103 driver)
- `simpleFocDriver/lib/SimpleFOCDrivers/src/comms/streams/BinaryIO.{h,cpp}`

## Executive Summary

The motor link communication uses a binary protocol (BinaryIO) with 0xA5 marker framing. The observed issue shows the parser receiving spurious `reg=0x00` (STATUS) responses with unexpected values (0x93, 0xB4, 0xF0) when expecting ACKs for registry commands like ENABLE (0x04) or CONTROL_MODE (0x05).

**Root cause candidates (in order of likelihood):**
1. Telemetry packets being misinterpreted as register responses due to frame synchronization loss
2. Marker byte (0xA5) appearing within float telemetry data causing re-sync at wrong position
3. Race condition between high-frequency telemetry (500Hz) and registry command responses

---

## Protocol Overview

### Packet Format
```
[0xA5][size][type][payload...]
  |      |    |
  |      |    +-- Packet type: 'R'=request, 'r'=response, 'T'=telemetry
  |      +------- Size = 1 + payload_length
  +-------------- Marker byte (sync)
```

### Register Response Format
```
[0xA5][size]['r'][reg_id][value_bytes...]
```

### Telemetry Format
```
[0xA5][size]['T'][motor_id][velocity:4bytes][status:1byte]
```

---

## Issues Identified

### Issue 1: Frame Synchronization Vulnerability (HIGH)

**Location:** [motor_link.c:346-401](Drivers/motors/motor_link.c#L346-L401)

The parser scans for 0xA5 marker to synchronize. However, 0xA5 (decimal 165) can legitimately appear within IEEE 754 float values in telemetry data.

```c
// motor_link_parser_try_parse() - line 357
while (idx < parser->len && parser->buf[idx] != MOTOR_LINK_BIN_MARKER)
{
    idx++;
}
```

**Problem:** When the parser loses sync (e.g., due to buffer overflow or UART error), it searches for 0xA5. If a telemetry float contains 0xA5 as a byte, the parser re-syncs at the wrong position:

```
Telemetry packet (correct):
[A5][07][54][00][xx][A5][yy][zz][ss]
                     ^^
                     This 0xA5 is part of velocity float!

Parser re-syncs here and interprets:
[A5][yy][zz] as a new packet with type=zz
```

**Evidence from logs:** The values 0x93, 0xB4, 0xF0, 0x75, 0xC0 appear as "status" bytes. These are NOT valid motor status values (which should be 0-7 typically). They are likely fragments of IEEE 754 floats being misinterpreted.

### Issue 2: Missing Checksum/CRC (HIGH)

**Location:** Both sides of protocol

The BinaryIO protocol has NO checksum or CRC. This means:
- Corrupted packets are not detected
- Partial packets can be interpreted as valid
- UART errors (noise, overruns) silently corrupt data

### Issue 3: Telemetry/Response Race Condition (MEDIUM)

**Location:** [motor_link.c:768-806](Drivers/motors/motor_link.c#L768-L806)

The `motor_link_port_wait_ack()` function polls for ACK while telemetry packets are streaming at 500Hz.

```c
while ((HAL_GetTick() - start) < timeout_ms)
{
    motor_link_port_pump_rx(port);
    if (port->ack_seq != last_seq)
    {
        // ACK received - but might be wrong register!
        if (port->last_ack.valid && port->last_ack.reg == reg)
        {
            return port->last_ack.ok != 0U;
        }
    }
    HAL_Delay(1);
}
```

**Problem:** The function only checks if the ACK register matches the expected register. If a spurious ACK arrives for a different register (e.g., STATUS), it updates `last_ack` but the wait continues. Multiple spurious ACKs can arrive before the real ACK, potentially causing timeout.

### Issue 4: Echo Mode Interleaving (LOW)

**Location:** [comms_streams.cpp:165](../../../simpleFocDriver/src/comms_streams.cpp#L165)

```cpp
static BootPacketCommander packet_commander(/*echo=*/true);
```

Echo mode is enabled, meaning every register write triggers a response.

- Telemetry: ~7 bytes * 500Hz = 3500 bytes/sec
- At 460800 baud: ~46080 bytes/sec capacity (~7.6% utilization)
- Registry responses: additional 6-8 bytes each

**Assessment:** At 460800 baud, bandwidth is not a concern. TX buffer overflow is unlikely. However, echo mode does add extra packets that the parser must handle, slightly increasing the chance of sync issues if other problems exist.

### Issue 5: ACK Matching Logic Gap (MEDIUM)

**Location:** [motor_link.c:403-433](Drivers/motors/motor_link.c#L403-L433)

```c
static bool motor_link_ack_matches_expected(const motor_link_ack_t *ack,
                                            const motor_link_pending_t *pending)
{
    if (!pending->active || pending->reg != ack->reg)
    {
        return false;  // Register mismatch - but ACK still logged!
    }
    // ...
}
```

When an ACK arrives for the wrong register, the function returns `false` but the ACK is still processed and logged. There's no mechanism to discard spurious ACKs or queue expected ACKs.

### Issue 6: Parser Buffer Drop Handling (LOW)

**Location:** [motor_link.c:334-343](Drivers/motors/motor_link.c#L334-L343)

```c
if (parser->len + len > MOTOR_LINK_PARSER_BUFFER_BYTES)
{
    size_t drop = (parser->len + len) - MOTOR_LINK_PARSER_BUFFER_BYTES;
    parser->drops += drop;
    memmove(parser->buf, parser->buf + drop, parser->len - drop);
    parser->len -= drop;
}
```

When the buffer overflows, bytes are dropped from the **beginning**. This can corrupt a partially received packet, leading to sync loss on the next parse attempt.

---

## Observed Behavior Analysis

From the debug output:
```
[APP][INFO] Motor link: left RX reg=0x00 len=1
[APP][INFO] RESP (1): 93
```

This pattern suggests:
1. Parser found 0xA5 marker (possibly within float data)
2. Next byte interpreted as size (likely small value)
3. Next byte interpreted as type (happened to be 0x72 = 'r')
4. Next byte interpreted as reg_id (0x00 = STATUS)
5. Remaining bytes interpreted as value (0x93)

The values 0x93, 0xB4, 0xF0 are likely exponent/mantissa bytes from velocity floats.

---

## Recommendations

### Immediate Fixes

1. **Add sequence number to ACK matching**
   ```c
   typedef struct {
       uint8_t active;
       uint8_t reg;
       uint8_t seq;  // Add sequence number
       // ...
   } motor_link_pending_t;
   ```

2. **Implement packet timeout/discard for spurious ACKs**
   ```c
   // In motor_link_port_handle_ack()
   if (!expected) {
       // Discard ACK if no matching pending request
       return;
   }
   ```

3. **Reduce telemetry rate during configuration**
   ```c
   // Before sending registry commands
   motor_link_port_set_telemetry_downsample(&port, 10000, timeout, false);
   // ... send commands ...
   // Restore rate after
   motor_link_port_set_telemetry_rate(&port, MOTOR_LINK_TELEM_RATE_HZ, timeout);
   ```

### Medium-Term Fixes

4. **Add simple checksum to protocol**
   - XOR checksum is cheap: `checksum ^= byte` for each byte
   - Add as last byte of packet
   - Requires firmware update on both sides

5. **Use escape sequence for marker byte**
   - If 0xA5 appears in payload, escape it (e.g., 0xA5 0x00)
   - Prevents false sync

6. **Implement proper frame buffer with state machine**
   ```c
   typedef enum {
       PARSE_IDLE,
       PARSE_SIZE,
       PARSE_TYPE,
       PARSE_PAYLOAD,
       PARSE_COMPLETE
   } parse_state_t;
   ```

### Troubleshooting Steps

To identify the exact root cause:

1. **Capture raw UART bytes**
   ```c
   // Add to motor_link_parser_feed()
   APP_LOG_DEBUG("RAW RX: %02X %02X %02X ...", data[0], data[1], data[2]);
   ```

2. **Log parser sync events**
   ```c
   // In motor_link_parser_try_parse(), after finding marker
   if (idx > 0U) {
       APP_LOG_WARN("Parser dropped %u bytes to re-sync", idx);
   }
   ```

3. **Monitor buffer state**
   ```c
   APP_LOG_DEBUG("Parser buf: len=%u drops=%u", parser->len, parser->drops);
   ```

4. **Disable telemetry temporarily**
   - Set `MOTOR_LINK_CONFIG_DOWNSAMPLE` to very large value (e.g., 100000)
   - Test registry commands in isolation
   - If issue disappears, confirms telemetry interference

5. **Check for UART errors on H7 side**
   ```c
   if (huart->ErrorCode != HAL_UART_ERROR_NONE) {
       APP_LOG_ERROR("UART error: 0x%lx", huart->ErrorCode);
   }
   ```

6. **Verify simpleFocDriver TX buffer**
   ```cpp
   // In handle_streams()
   if (uart_dma_stream().tx_drops() > 0) {
       log_packet(LOG_WARN, "UART", "TX drops");
   }
   ```

---

## Test Plan

1. **Isolated registry test**
   - Disable telemetry completely
   - Send single ENABLE command
   - Verify ACK received correctly

2. **Telemetry stress test**
   - Enable telemetry at maximum rate
   - Monitor for parser drops/sync loss

3. **Combined test**
   - Telemetry running at 500Hz
   - Send registry command every 100ms
   - Log success rate

4. **Noise injection test**
   - Add artificial delays or byte corruption
   - Verify error handling

---

## Files to Modify

| File | Change |
|------|--------|
| `motor_link.c` | Add sequence numbers, improve ACK filtering |
| `motor_link.h` | Add debug API for parser stats |
| `comms_streams.cpp` | Consider disabling echo or adding sequence |
| `BinaryIO.cpp` | (Future) Add checksum support |

---

## Conclusion

The most likely root cause is **frame synchronization loss due to 0xA5 appearing in float telemetry data**, combined with **lack of checksum validation**. The high-frequency telemetry stream (500Hz) increases the probability of this occurring.

Recommended immediate action: Reduce telemetry rate during configuration phase and add logging to confirm sync loss events.
