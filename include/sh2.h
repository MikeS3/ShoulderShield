/*
 * Copyright 2015-2021 CEVA, Inc. - Modified for Multi-Instance Support
 */

#ifndef SH2_H
#define SH2_H

#include <stdint.h>
#include <stdbool.h>
#include "sh2_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration of SH2 instance structure
typedef struct sh2_Instance_s sh2_Instance_t;

/***************************************************************************************
 * Public type definitions (unchanged from original)
 ***************************************************************************************/

#define SH2_MAX_SENSOR_EVENT_LEN (60)
typedef struct sh2_SensorEvent {
    uint64_t timestamp_uS;
    int64_t delay_uS;
    uint8_t len;
    uint8_t reportId;
    uint8_t report[SH2_MAX_SENSOR_EVENT_LEN];
} sh2_SensorEvent_t;

typedef void (sh2_SensorCallback_t)(void * cookie, sh2_SensorEvent_t *pEvent);

typedef struct sh2_ProductId_s {
    uint8_t resetCause;
    uint8_t swVersionMajor;
    uint8_t swVersionMinor;
    uint32_t swPartNumber;
    uint32_t swBuildNumber;
    uint16_t swVersionPatch;
    uint8_t reserved0;
    uint8_t reserved1;
} sh2_ProductId_t;

#define SH2_MAX_PROD_ID_ENTRIES (5)
typedef struct sh2_ProductIds_s {
    sh2_ProductId_t entry[SH2_MAX_PROD_ID_ENTRIES];
    uint8_t numEntries;
} sh2_ProductIds_t;

enum sh2_SensorId_e {
    SH2_RAW_ACCELEROMETER = 0x14,
    SH2_ACCELEROMETER = 0x01,
    SH2_LINEAR_ACCELERATION = 0x04,
    SH2_GRAVITY = 0x06,
    SH2_RAW_GYROSCOPE = 0x15,
    SH2_GYROSCOPE_CALIBRATED = 0x02,
    SH2_GYROSCOPE_UNCALIBRATED = 0x07,
    SH2_RAW_MAGNETOMETER = 0x16,
    SH2_MAGNETIC_FIELD_CALIBRATED = 0x03,
    SH2_MAGNETIC_FIELD_UNCALIBRATED = 0x0f,
    SH2_ROTATION_VECTOR = 0x05,
    SH2_GAME_ROTATION_VECTOR = 0x08,
    SH2_GEOMAGNETIC_ROTATION_VECTOR = 0x09,
    SH2_PRESSURE = 0x0a,
    SH2_AMBIENT_LIGHT = 0x0b,
    SH2_HUMIDITY = 0x0c,
    SH2_PROXIMITY = 0x0d,
    SH2_TEMPERATURE = 0x0e,
    SH2_RESERVED = 0x17,
    SH2_TAP_DETECTOR = 0x10,
    SH2_STEP_DETECTOR = 0x18,
    SH2_STEP_COUNTER = 0x11,
    SH2_SIGNIFICANT_MOTION = 0x12,
    SH2_STABILITY_CLASSIFIER = 0x13,
    SH2_SHAKE_DETECTOR = 0x19,
    SH2_FLIP_DETECTOR = 0x1a,
    SH2_PICKUP_DETECTOR = 0x1b,
    SH2_STABILITY_DETECTOR = 0x1c,
    SH2_PERSONAL_ACTIVITY_CLASSIFIER = 0x1e,
    SH2_SLEEP_DETECTOR = 0x1f,
    SH2_TILT_DETECTOR = 0x20,
    SH2_POCKET_DETECTOR = 0x21,
    SH2_CIRCLE_DETECTOR = 0x22,
    SH2_HEART_RATE_MONITOR = 0x23,
    SH2_ARVR_STABILIZED_RV = 0x28,
    SH2_ARVR_STABILIZED_GRV = 0x29,
    SH2_GYRO_INTEGRATED_RV = 0x2A,
    SH2_IZRO_MOTION_REQUEST = 0x2B,
    SH2_RAW_OPTICAL_FLOW = 0x2C,
    SH2_DEAD_RECKONING_POSE = 0x2D,
    SH2_WHEEL_ENCODER = 0x2E,
    SH2_MAX_SENSOR_ID = 0x2E,
};
typedef uint8_t sh2_SensorId_t;

typedef struct sh2_SensorConfig {
    bool changeSensitivityEnabled;
    bool changeSensitivityRelative;
    bool wakeupEnabled;
    bool alwaysOnEnabled;
    bool sniffEnabled;
    uint16_t changeSensitivity;
    uint32_t reportInterval_us;
    uint32_t batchInterval_us;
    uint32_t sensorSpecific;
} sh2_SensorConfig_t;

typedef struct sh2_SensorMetadata {
    uint8_t meVersion;
    uint8_t mhVersion;
    uint8_t shVersion;
    uint32_t range;
    uint32_t resolution;
    uint16_t revision;
    uint16_t power_mA;
    uint32_t minPeriod_uS;
    uint32_t maxPeriod_uS;
    uint32_t fifoReserved;
    uint32_t fifoMax;
    uint32_t batchBufferBytes;
    uint16_t qPoint1;
    uint16_t qPoint2;
    uint16_t qPoint3;
    uint32_t vendorIdLen;
    char vendorId[48];
    uint32_t sensorSpecificLen;
    uint8_t sensorSpecific[48];
} sh2_SensorMetadata_t;

typedef struct sh2_ErrorRecord {
    uint8_t severity;
    uint8_t sequence;
    uint8_t source;
    uint8_t error;
    uint8_t module;
    uint8_t code;
} sh2_ErrorRecord_t;

typedef struct sh2_Counts {
    uint32_t offered;
    uint32_t accepted;
    uint32_t on;
    uint32_t attempted;
} sh2_Counts_t;

typedef enum sh2_TareBasis {
    SH2_TARE_BASIS_ROTATION_VECTOR = 0,
    SH2_TARE_BASIS_GAMING_ROTATION_VECTOR = 1,
    SH2_TARE_BASIS_GEOMAGNETIC_ROTATION_VECTOR = 2,
} sh2_TareBasis_t;

typedef enum sh2_TareAxis {
    SH2_TARE_X = 1,
    SH2_TARE_Y = 2,
    SH2_TARE_Z = 4,
    SH2_TARE_CONTROL_VECTOR_X = (1 << 3),
    SH2_TARE_CONTROL_VECTOR_Y = (0 << 3),
    SH2_TARE_CONTROL_VECTOR_Z = (2 << 3),
    SH2_TARE_CONTROL_SEQUENCE_DEFAULT = (0 << 5),
    SH2_TARE_CONTROL_SEQUENCE_PRE = (1 << 5),
    SH2_TARE_CONTROL_SEQUENCE_POST = (2 << 5),
} sh2_TareAxis_t;

typedef struct sh2_Quaternion {
    double x;
    double y;
    double z;
    double w;
} sh2_Quaternion_t;

typedef enum {
    SH2_OSC_INTERNAL    = 0,
    SH2_OSC_EXT_CRYSTAL = 1,
    SH2_OSC_EXT_CLOCK   = 2,
} sh2_OscType_t;

typedef enum {
    SH2_CAL_SUCCESS = 0,
    SH2_CAL_NO_ZRO,
    SH2_CAL_NO_STATIONARY_DETECTION,
    SH2_CAL_ROTATION_OUTSIDE_SPEC,
    SH2_CAL_ZRO_OUTSIDE_SPEC,
    SH2_CAL_ZGO_OUTSIDE_SPEC,
    SH2_CAL_GYRO_GAIN_OUTSIDE_SPEC,
    SH2_CAL_GYRO_PERIOD_OUTSIDE_SPEC,
    SH2_CAL_GYRO_DROPS_OUTSIDE_SPEC,
} sh2_CalStatus_t;

enum sh2_AsyncEventId_e {
    SH2_RESET,
    SH2_SHTP_EVENT,
    SH2_GET_FEATURE_RESP,
};
typedef enum sh2_AsyncEventId_e sh2_AsyncEventId_t;

enum sh2_ShtpEvent_e {
    SH2_SHTP_TX_DISCARD = 0,
    SH2_SHTP_SHORT_FRAGMENT = 1,
    SH2_SHTP_TOO_LARGE_PAYLOADS = 2,
    SH2_SHTP_BAD_RX_CHAN = 3,
    SH2_SHTP_BAD_TX_CHAN = 4,
    SH2_SHTP_BAD_FRAGMENT = 5,
    SH2_SHTP_BAD_SN = 6,
    SH2_SHTP_INTERRUPTED_PAYLOAD = 7,
};
typedef uint8_t sh2_ShtpEvent_t;

typedef struct sh2_SensorConfigResp_e {
    sh2_SensorId_t sensorId;
    sh2_SensorConfig_t sensorConfig;
} sh2_SensorConfigResp_t;

typedef struct sh2_AsyncEvent {
    uint32_t eventId;
    union {
        sh2_ShtpEvent_t shtpEvent;
        sh2_SensorConfigResp_t sh2SensorConfigResp;
    };
} sh2_AsyncEvent_t;

typedef void (sh2_EventCallback_t)(void * cookie, sh2_AsyncEvent_t *pEvent);

typedef enum {
    SH2_IZRO_MI_UNKNOWN = 0,
    SH2_IZRO_MI_STATIONARY_NO_VIBRATION,
    SH2_IZRO_MI_STATIONARY_WITH_VIBRATION,
    SH2_IZRO_MI_IN_MOTION,
    SH2_IZRO_MI_ACCELERATING,
} sh2_IZroMotionIntent_t;

typedef enum {
    SH2_IZRO_MR_NO_REQUEST = 0,
    SH2_IZRO_MR_STAY_STATIONARY,
    SH2_IZRO_MR_STATIONARY_NON_URGENT,
    SH2_IZRO_MR_STATIONARY_URGENT,
} sh2_IZroMotionRequest_t;

/***************************************************************************************
 * Multi-Instance Public API
 **************************************************************************************/

/**
 * @brief Create a new SH2 instance
 * @return Pointer to new SH2 instance, or NULL on failure
 */
sh2_Instance_t* sh2_createInstance(void);

/**
 * @brief Destroy an SH2 instance
 * @param pInstance Pointer to SH2 instance
 */
void sh2_destroyInstance(sh2_Instance_t* pInstance);

/**
 * @brief Open a session with a sensor hub (instance-specific)
 */
int sh2_openInstance(sh2_Instance_t* pInstance, sh2_Hal_t *pHal,
                     sh2_EventCallback_t *eventCallback, void *eventCookie);

/**
 * @brief Close a session with a sensor hub (instance-specific)
 */
void sh2_closeInstance(sh2_Instance_t* pInstance);

/**
 * @brief Service the SH2 device (instance-specific)
 */
void sh2_serviceInstance(sh2_Instance_t* pInstance);

/**
 * @brief Register a function to receive sensor events (instance-specific)
 */
int sh2_setSensorCallbackInstance(sh2_Instance_t* pInstance, 
                                  sh2_SensorCallback_t *callback, void *cookie);

/**
 * @brief Set sensor configuration (instance-specific)
 */
int sh2_setSensorConfigInstance(sh2_Instance_t* pInstance, 
                                sh2_SensorId_t sensorId, const sh2_SensorConfig_t *pConfig);

/**
 * @brief Get Product ID information (instance-specific)
 */
int sh2_getProdIdsInstance(sh2_Instance_t* pInstance, sh2_ProductIds_t *prodIds);

/***************************************************************************************
 * Legacy Global API (for backward compatibility - uses default instance)
 **************************************************************************************/

int sh2_open(sh2_Hal_t *pHal, sh2_EventCallback_t *eventCallback, void *eventCookie);
void sh2_close(void);
void sh2_service(void);
int sh2_setSensorCallback(sh2_SensorCallback_t *callback, void *cookie);
int sh2_setSensorConfig(sh2_SensorId_t sensorId, const sh2_SensorConfig_t *pConfig);
int sh2_getProdIds(sh2_ProductIds_t *prodIds);

// Other legacy functions...
int sh2_devReset(void);
int sh2_devOn(void);
int sh2_devSleep(void);
int sh2_getSensorConfig(sh2_SensorId_t sensorId, sh2_SensorConfig_t *pConfig);
int sh2_getMetadata(sh2_SensorId_t sensorId, sh2_SensorMetadata_t *pData);

// FRS Record IDs and other constants (unchanged)
#define STATIC_CALIBRATION_AGM                   (0x7979)
#define NOMINAL_CALIBRATION                      (0x4D4D)
#define STATIC_CALIBRATION_SRA                   (0x8A8A)
#define NOMINAL_CALIBRATION_SRA                  (0x4E4E)
#define DYNAMIC_CALIBRATION                      (0x1F1F)
// ... (other FRS constants)

// Calibration flags
#define SH2_CAL_ACCEL (0x01)
#define SH2_CAL_GYRO  (0x02)
#define SH2_CAL_MAG   (0x04)
#define SH2_CAL_PLANAR (0x08)
#define SH2_CAL_ON_TABLE (0x10)

#ifdef __cplusplus
}
#endif

#endif // SH2_H