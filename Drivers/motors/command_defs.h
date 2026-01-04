#ifndef COMMAND_DEFS_H
#define COMMAND_DEFS_H

/*
 * Robust Binary I/O v2 - Command Definitions
 *
 * Consolidated command structure using 'C' request / 'c' response packet types.
 * See docs/RobustBinaryIOv2.md for full protocol specification.
 */

/* Command IDs */
#define CMD_WRITE       0x01U  /* Save settings to flash */
#define CMD_CALIBRATE   0x02U  /* Run sensor calibration */
#define CMD_BOOTLOADER  0x03U  /* Enter bootloader mode */

/* Command response status codes */
#define CMD_STATUS_OK      0x00U  /* Success */
#define CMD_STATUS_ERROR   0x01U  /* Error */
#define CMD_STATUS_BUSY    0x02U  /* Busy (e.g., calibration in progress) */
#define CMD_STATUS_UNKNOWN 0xFFU  /* Unknown command */

/* Packet types */
#define PKT_CMD_REQUEST   'C'  /* 0x43 - Command request */
#define PKT_CMD_RESPONSE  'c'  /* 0x63 - Command response */

/* Register packet types */
#define PKT_REG_REQUEST   'R'  /* 0x52 - Register request */
#define PKT_REG_RESPONSE  'r'  /* 0x72 - Register response */

/* Telemetry packet type */
#define PKT_TELEMETRY     'T'  /* 0x54 - Telemetry data */

/* Log packet type */
#define PKT_LOG           'L'  /* 0x4C - Log message */

#endif /* COMMAND_DEFS_H */
