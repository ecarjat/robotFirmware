#ifndef BLACKBOX_FORMAT_H
#define BLACKBOX_FORMAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Flash Memory Layout */
#define LOG_META_START      0x000000U   /* Metadata start address */
#define LOG_META_SIZE       0x002000U   /* Two 4 KB metadata sectors */
#define LOG_RING_START      0x002000U   /* Ring buffer start */
#define LOG_RING_SIZE       0x7FE000U   /* ~8184 KB ring buffer */
#define LOG_RING_END        (LOG_RING_START + LOG_RING_SIZE)

/* Metadata dual-slot layout */
#define LOG_META_SLOT0      0x000000U   /* Slot 0: 0x000000-0x000FFF (4 KB) */
#define LOG_META_SLOT1      0x001000U   /* Slot 1: 0x001000-0x001FFF (4 KB) */
#define LOG_META_SLOT_SIZE  0x001000U   /* 4 KB per slot / erase sector */

/* Log field bitmask (runtime configurable) */
#define LOGF_IMU_RAW        (1u << 0)   /* acc_raw + gyro_raw from active IMU */
#define LOGF_IMU2_HEALTH    (1u << 1)   /* dual-IMU health metrics */
#define LOGF_EKF            (1u << 2)   /* EKF state (theta, thetaDot, biases, x, xDot) */
#define LOGF_WHEELS         (1u << 3)   /* wheel velocities (wL, wR, v) */
#define LOGF_PID            (1u << 4)   /* PID internals (P/I/D, u_common, u_turn, uL, uR) */
#define LOGF_POWER          (1u << 5)   /* Power metrics (Vbat) - reserved for future */
#define LOGF_DIAG           (1u << 6)   /* Diagnostic counters - reserved for future */

/* Default log fields (enable IMU, health, EKF, wheels, PID) */
#define LOG_FIELDS_MASK_DEFAULT  (LOGF_IMU_RAW | LOGF_IMU2_HEALTH | \
                                   LOGF_EKF | LOGF_WHEELS | LOGF_PID)

/* Record constants */
#define LOG_RECORD_MAGIC    0xA55AU
#define LOG_RECORD_VERSION  1U
#define LOG_RECORD_SIZE     160U        /* Fixed record size (pad to 160 bytes) */

/* Meta constants */
#define LOG_META_MAGIC      "R2WLOG1"   /* 7 chars + null terminator = 8 bytes */
#define LOG_META_VERSION    1U

/* Record flags bits */
#define LOGF_REC_ACCEL_GATED   (1u << 0)  /* Accelerometer gated due to vibration */
#define LOGF_REC_IMU_FALLBACK  (1u << 1)  /* Using secondary IMU */
#define LOGF_REC_UL_SAT        (1u << 2)  /* Left motor saturated */
#define LOGF_REC_UR_SAT        (1u << 3)  /* Right motor saturated */
#define LOGF_REC_FALLEN        (1u << 4)  /* Robot in FALLEN state */
#define LOGF_REC_ARMED         (1u << 5)  /* Robot is armed */
#define LOGF_REC_LQR_ACTIVE    (1u << 6)  /* LQR inner loop active (vs PID) */

/**
 * @brief Log metadata structure (stored in flash at LOG_META)
 *
 * Uses dual-slot redundancy for power-loss tolerance.
 * Slots alternate with incrementing sequence number.
 * On boot, choose newest valid slot by sequence + CRC.
 */
typedef struct __attribute__((packed)) {
  uint8_t  magic[8];           /* "R2WLOG1\0" */
  uint16_t version;            /* LOG_META_VERSION */
  uint16_t record_size;        /* sizeof(LogRecord) = 160 */
  uint16_t rate_hz;            /* Log rate (typically 400 Hz) */
  uint32_t log_fields_mask;    /* Active LOGF_* fields */
  uint16_t reserved;
  uint32_t ring_start;         /* LOG_RING_START = 0x002000 */
  uint32_t ring_size;          /* LOG_RING_SIZE */
  uint32_t write_addr;         /* Next write position in ring */
  uint32_t wrap_count;         /* Number of ring wraps */
  uint32_t boot_count;         /* Increments on each boot */
  uint32_t last_dump_id;       /* Last dump file number */
  uint32_t sequence;           /* Slot sequence (for dual-slot selection) */
  uint32_t meta_crc32;         /* CRC32 of all fields except this one */
} LogMeta;  /* ~68 bytes */

/**
 * @brief Log record structure (stored in ring buffer)
 *
 * Fixed-size record (160 bytes) for deterministic ring buffer operation.
 * Fields are conditionally populated based on log_fields_mask.
 * Unpopulated fields are zero-filled.
 */
typedef struct __attribute__((packed)) {
  /* Header (always present) */
  uint16_t magic;              /* LOG_RECORD_MAGIC = 0xA55A */
  uint8_t  version;            /* LOG_RECORD_VERSION = 1 */
  uint8_t  flags;              /* LOGF_REC_* status flags */
  uint32_t seq;                /* Sequence number (increments each tick) */
  uint32_t t_us;               /* Timestamp in microseconds (wraps at ~4295s) */
  uint8_t  active_imu;         /* 0=BMI270, 1=ICM42688 */
  uint8_t  reserved[3];        /* Padding for alignment */

  /* IMU raw data (LOGF_IMU_RAW) - from active sensor used by EKF */
  int16_t  acc_raw[3];         /* Raw accelerometer [x,y,z] in sensor units */
  int16_t  gyro_raw[3];        /* Raw gyroscope [x,y,z] in sensor units */

  /* Dual-IMU health metrics (LOGF_IMU2_HEALTH) */
  float    gyro_diff_dps;      /* 3D gyro disagreement (deg/s) */
  float    acc_angle_deg;      /* Accelerometer angle disagreement (deg) */
  float    vib_grms;           /* Vibration RMS (g) */

  /* EKF state outputs (LOGF_EKF) */
  float    theta_rad;          /* Pitch angle (rad) */
  float    thetaDot_rads;      /* Pitch rate (rad/s) */
  float    gyro_bias_rads;     /* EKF gyro bias estimate (rad/s) */
  float    x_m;                /* Position estimate (m) */
  float    x_dot_mps;          /* Velocity estimate (m/s) */

  /* Wheel velocities (LOGF_WHEELS) */
  float    wL_rads;            /* Left wheel angular velocity (rad/s) */
  float    wR_rads;            /* Right wheel angular velocity (rad/s) */
  float    v_mps;              /* Linear velocity from encoders (m/s) */

  /* PID controller internals (LOGF_PID) */
  float    theta_ref_rad;      /* Pitch target from velocity PID (rad) */
  float    e_theta_rad;        /* Pitch error (rad) */
  float    P;                  /* Proportional term */
  float    I;                  /* Integral term */
  float    D;                  /* Derivative term */
  float    u_common;           /* Common drive command (u_sum_cmd, Nm) */
  float    u_turn;             /* Turn differential command (Nm) */
  float    uL_cmd;             /* Left motor command (Nm) */
  float    uR_cmd;             /* Right motor command (Nm) */
  float    u_sum_lqr;          /* LQR u_sum before blending (Nm) */
  uint8_t  lqr_alpha;          /* LQR blend factor (0-255 = 0.0-1.0) */
  uint8_t  pad[3];             /* Padding to maintain alignment */

  /* Trailer */
  uint32_t crc32;              /* CRC32 of entire record excluding this field */
} LogRecord;  /* 160 bytes with LQR fields */

/* Compile-time size validation */
_Static_assert(sizeof(LogRecord) <= LOG_RECORD_SIZE,
               "LogRecord exceeds LOG_RECORD_SIZE");

/**
 * @brief Dump file trailer (appended to SD dump files)
 *
 * Provides dump metadata and integrity check.
 */
typedef struct __attribute__((packed)) {
  uint32_t start_seq;          /* First record sequence number */
  uint32_t end_seq;            /* Last record sequence number */
  uint32_t record_count;       /* Number of records in dump */
  uint32_t dump_crc32;         /* CRC32 of all records in dump */
} LogDumpTrailer;  /* 16 bytes */

#ifdef __cplusplus
}
#endif

#endif /* BLACKBOX_FORMAT_H */
