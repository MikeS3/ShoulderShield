/*
 * Copyright 2015-2022 CEVA, Inc. - Modified for Multi-Instance Support
 */

#include "sh2.h"
#include "sh2_err.h"
#include "shtp.h"
#include "sh2_util.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ------------------------------------------------------------------------
// Private type definitions

#define CHAN_EXECUTABLE_DEVICE    (1)
#define CHAN_SENSORHUB_CONTROL    (2)
#define CHAN_SENSORHUB_INPUT      (3)
#define CHAN_SENSORHUB_INPUT_WAKE (4)
#define CHAN_SENSORHUB_INPUT_GIRV (5)

// executable/device channel responses
#define EXECUTABLE_DEVICE_CMD_RESET (1)
#define EXECUTABLE_DEVICE_CMD_ON    (2)
#define EXECUTABLE_DEVICE_CMD_SLEEP (3)
#define EXECUTABLE_DEVICE_RESP_RESET_COMPLETE (1)

// Command and Subcommand values
#define SH2_CMD_ERRORS                 1
#define SH2_CMD_COUNTS                 2
#define     SH2_COUNTS_GET_COUNTS          0
#define     SH2_COUNTS_CLEAR_COUNTS        1
#define SH2_CMD_TARE                   3
#define     SH2_TARE_TARE_NOW              0
#define     SH2_TARE_PERSIST_TARE          1
#define     SH2_TARE_SET_REORIENTATION     2
#define SH2_CMD_INITIALIZE             4
#define     SH2_INIT_SYSTEM                1
#define     SH2_INIT_UNSOLICITED           0x80
#define SH2_CMD_DCD                    6
#define SH2_CMD_ME_CAL                 7
#define SH2_CMD_DCD_SAVE               9
#define SH2_CMD_GET_OSC_TYPE           0x0A
#define SH2_CMD_CLEAR_DCD_AND_RESET    0x0B
#define SH2_CMD_CAL                    0x0C
#define     SH2_CAL_START                   0
#define     SH2_CAL_FINISH                  1
#define SH2_CMD_INTERACTIVE_ZRO        0x0E
#define SH2_CMD_WHEEL_REQ              0x0F
#define SH2_CMD_DR_CAL_SAVE            0x10

// Report IDs
#define SENSORHUB_COMMAND_REQ          (0xF2)
#define SENSORHUB_COMMAND_RESP         (0xF1)
#define SENSORHUB_PROD_ID_REQ          (0xF9)
#define SENSORHUB_PROD_ID_RESP         (0xF8)
#define SENSORHUB_GET_FEATURE_REQ      (0xFE)
#define SENSORHUB_GET_FEATURE_RESP     (0xFC)
#define SENSORHUB_SET_FEATURE_CMD      (0xFD)
#define SENSORHUB_BASE_TIMESTAMP_REF   (0xFB)
#define SENSORHUB_TIMESTAMP_REBASE     (0xFA)
#define SENSORHUB_FORCE_SENSOR_FLUSH   (0xF0)
#define SENSORHUB_FLUSH_COMPLETED      (0xEF)

#define COMMAND_PARAMS (9)
#define RESPONSE_VALUES (11)
#define MAX_FRS_WORDS (72)
#define ADVERT_TIMEOUT_US (200000)

// Feature flags
#define FEAT_CHANGE_SENSITIVITY_ENABLED    (1 << 1)
#define FEAT_CHANGE_SENSITIVITY_RELATIVE   (1 << 0)
#define FEAT_WAKE_ENABLED                  (1 << 2)
#define FEAT_ALWAYS_ON_ENABLED             (1 << 3)
#define FEAT_SNIFF_ENABLED                 (1 << 4)

// Packed structures
#if defined(_MSC_VER)
#define PACKED_STRUCT struct
#pragma pack(push, 1)
#elif defined(__GNUC__)
#define PACKED_STRUCT struct __attribute__((packed))
#else 
#define PACKED_STRUCT __packed struct
#endif

typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t seq;
    uint8_t command;
    uint8_t p[COMMAND_PARAMS];
} CommandReq_t;

typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t seq;
    uint8_t command;
    uint8_t commandSeq;
    uint8_t respSeq;
    uint8_t r[RESPONSE_VALUES];
} CommandResp_t;

typedef PACKED_STRUCT {
    uint8_t reportId;  
    uint8_t reserved;
} ProdIdReq_t;

typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t resetCause;
    uint8_t swVerMajor;
    uint8_t swVerMinor;
    uint32_t swPartNumber;
    uint32_t swBuildNumber;
    uint16_t swVerPatch;
    uint8_t reserved0;
    uint8_t reserved1;
} ProdIdResp_t;

typedef PACKED_STRUCT{
    uint8_t reportId;
    uint8_t featureReportId;
} GetFeatureReq_t;

typedef PACKED_STRUCT{
    uint8_t reportId;
    uint8_t featureReportId;
    uint8_t flags;
    uint16_t changeSensitivity;
    uint32_t reportInterval_uS;
    uint32_t batchInterval_uS;
    uint32_t sensorSpecific;
} GetFeatureResp_t;

typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t featureReportId;
    uint8_t flags;
    uint16_t changeSensitivity;
    uint32_t reportInterval_uS;
    uint32_t batchInterval_uS;
    uint32_t sensorSpecific;
} SetFeatureReport_t;

// SH2 Instance structure (replaces global singleton)
struct sh2_Instance_s {
    // Pointer to the SHTP HAL
    sh2_Hal_t *pHal;

    // associated SHTP instance
    void *pShtp;
    
    volatile bool resetComplete;

    // Event callback and it's cookie
    sh2_EventCallback_t *eventCallback;
    void *eventCookie;

    // Sensor callback and it's cookie
    sh2_SensorCallback_t *sensorCallback;
    void *sensorCookie;

    // Command sequence tracking
    uint8_t lastCmdId;
    uint8_t cmdSeq;
    uint8_t nextCmdSeq;

    // Storage for FRS data
    uint32_t frsData[MAX_FRS_WORDS];
    uint16_t frsDataLen;

    // Stats
    uint32_t execBadPayload;
    uint32_t emptyPayloads;
    uint32_t unknownReportIds;

    // Instance ID for debugging
    int instance_id;
};

// For backward compatibility - default instance
static sh2_Instance_t* default_instance = NULL;

// ------------------------------------------------------------------------
// Report length lookup table

typedef struct {
    uint8_t id;
    uint8_t len;
} sh2_ReportLen_t;

static const sh2_ReportLen_t sh2ReportLens[] = {
    {.id = SH2_ACCELEROMETER,                .len = 10},  
    {.id = SH2_GYROSCOPE_CALIBRATED,         .len = 10},
    {.id = SH2_MAGNETIC_FIELD_CALIBRATED,    .len = 10},
    {.id = SH2_LINEAR_ACCELERATION,          .len = 10},
    {.id = SH2_ROTATION_VECTOR,              .len = 14},
    {.id = SH2_GRAVITY,                      .len = 10},
    {.id = SH2_GYROSCOPE_UNCALIBRATED,       .len = 16},
    {.id = SH2_GAME_ROTATION_VECTOR,         .len = 12},
    {.id = SH2_GEOMAGNETIC_ROTATION_VECTOR,  .len = 14},
    {.id = SH2_PRESSURE,                     .len =  8},
    {.id = SH2_AMBIENT_LIGHT,                .len =  8},
    {.id = SH2_HUMIDITY,                     .len =  6},
    {.id = SH2_PROXIMITY,                    .len =  6},
    {.id = SH2_TEMPERATURE,                  .len =  6},
    {.id = SH2_MAGNETIC_FIELD_UNCALIBRATED,  .len = 16},
    {.id = SH2_TAP_DETECTOR,                 .len =  5},
    {.id = SH2_STEP_COUNTER,                 .len = 12},
    {.id = SH2_SIGNIFICANT_MOTION,           .len =  6},
    {.id = SH2_STABILITY_CLASSIFIER,         .len =  6},
    {.id = SH2_RAW_ACCELEROMETER,            .len = 16},
    {.id = SH2_RAW_GYROSCOPE,                .len = 16},
    {.id = SH2_RAW_MAGNETOMETER,             .len = 16},
    {.id = SH2_STEP_DETECTOR,                .len =  8},
    {.id = SH2_SHAKE_DETECTOR,               .len =  6},
    {.id = SH2_FLIP_DETECTOR,                .len =  6},
    {.id = SH2_PICKUP_DETECTOR,              .len =  8},
    {.id = SH2_STABILITY_DETECTOR,           .len =  6},
    {.id = SH2_PERSONAL_ACTIVITY_CLASSIFIER, .len = 16},
    {.id = SH2_SLEEP_DETECTOR,               .len =  6},
    {.id = SH2_TILT_DETECTOR,                .len =  6},
    {.id = SH2_POCKET_DETECTOR,              .len =  6},
    {.id = SH2_CIRCLE_DETECTOR,              .len =  6},
    {.id = SH2_HEART_RATE_MONITOR,           .len =  6},
    {.id = SH2_ARVR_STABILIZED_RV,           .len = 14},
    {.id = SH2_ARVR_STABILIZED_GRV,          .len = 12},
    {.id = SH2_GYRO_INTEGRATED_RV,           .len = 14},
    {.id = SH2_IZRO_MOTION_REQUEST,          .len =  6},
    {.id = SENSORHUB_FLUSH_COMPLETED,        .len =  2},
    {.id = SENSORHUB_COMMAND_RESP,           .len = 16},
    {.id = SENSORHUB_PROD_ID_RESP,           .len = 16},
    {.id = SENSORHUB_TIMESTAMP_REBASE,       .len =  5},
    {.id = SENSORHUB_BASE_TIMESTAMP_REF,     .len =  5},
    {.id = SENSORHUB_GET_FEATURE_RESP,       .len = 17},
};

// ------------------------------------------------------------------------
// Private utility functions

static uint8_t getReportLen(uint8_t reportId) {
    for (unsigned n = 0; n < sizeof(sh2ReportLens)/sizeof(sh2ReportLens[0]); n++) {
        if (sh2ReportLens[n].id == reportId) {
            return sh2ReportLens[n].len;
        }
    }
    return 0;
}

static int sendCtrl(sh2_Instance_t *pSh2, const uint8_t *data, uint16_t len) {
    return shtp_send(pSh2->pShtp, CHAN_SENSORHUB_CONTROL, data, len);
}

static uint64_t touSTimestamp(uint32_t hostInt, int32_t referenceDelta, uint16_t delay) {
    static uint32_t lastHostInt = 0;
    static uint32_t rollovers = 0;
    uint64_t timestamp;

    if (hostInt < lastHostInt) {
        rollovers++;
    }
    lastHostInt = hostInt;
    
    timestamp = ((uint64_t)rollovers << 32);
    timestamp += hostInt + (referenceDelta + delay) * 100;

    return timestamp;
}

// ------------------------------------------------------------------------
// SHTP Channel Handlers

static void sensorhubInputHdlr(sh2_Instance_t *pSh2, uint8_t *payload, uint16_t len, uint32_t timestamp) {
    sh2_SensorEvent_t event;
    uint16_t cursor = 0;
    int32_t referenceDelta = 0;

    while (cursor < len) {
        uint8_t reportId = payload[cursor];
        uint8_t reportLen = getReportLen(reportId);
        
        if (reportLen == 0) {
            pSh2->unknownReportIds++;
            return;
        }

        if (reportId == SENSORHUB_BASE_TIMESTAMP_REF) {
            // Handle timestamp reference
            cursor += reportLen;
            continue;
        }
        else if (reportId == SENSORHUB_TIMESTAMP_REBASE) {
            // Handle timestamp rebase
            cursor += reportLen;
            continue;
        }
        else {
            // Regular sensor event
            uint8_t *pReport = payload + cursor;
            uint16_t delay = ((pReport[2] & 0xFC) << 6) + pReport[3];
            event.timestamp_uS = touSTimestamp(timestamp, referenceDelta, delay);
            event.delay_uS = (referenceDelta + delay) * 100;
            event.reportId = reportId;
            memcpy(event.report, pReport, reportLen);
            event.len = reportLen;
            
            if (pSh2->sensorCallback != 0) {
                pSh2->sensorCallback(pSh2->sensorCookie, &event);
            }
        }
        
        cursor += reportLen;
    }
}

static void sensorhubInputNormalHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp) {
    sh2_Instance_t *pSh2 = (sh2_Instance_t *)cookie;
    sensorhubInputHdlr(pSh2, payload, len, timestamp);
}

static void sensorhubInputWakeHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp) {
    sh2_Instance_t *pSh2 = (sh2_Instance_t *)cookie;
    sensorhubInputHdlr(pSh2, payload, len, timestamp);
}

static void sensorhubInputGyroRvHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp) {
    sh2_Instance_t *pSh2 = (sh2_Instance_t *)cookie;
    sh2_SensorEvent_t event;
    uint16_t cursor = 0;

    uint8_t reportId = SH2_GYRO_INTEGRATED_RV;
    uint8_t reportLen = getReportLen(reportId);

    while (cursor < len) {
        event.timestamp_uS = timestamp;
        event.reportId = reportId;
        memcpy(event.report, payload + cursor, reportLen);
        event.len = reportLen;

        if (pSh2->sensorCallback != 0) {
            pSh2->sensorCallback(pSh2->sensorCookie, &event);
        }

        cursor += reportLen;
    }
}

static void sensorhubControlHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp) {
    (void)timestamp;
    sh2_Instance_t *pSh2 = (sh2_Instance_t *)cookie;

    if (len == 0) {
        pSh2->emptyPayloads++;
        return;
    }

    // Handle control responses here
    // This is a simplified version - full implementation would handle all control responses
}

static void executableDeviceHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp) {
    (void)timestamp;
    sh2_Instance_t *pSh2 = (sh2_Instance_t *)cookie;

    if (len != 1) {
        pSh2->execBadPayload++;
        return;
    }
    
    switch (payload[0]) {
        case EXECUTABLE_DEVICE_RESP_RESET_COMPLETE:
            pSh2->resetComplete = true;
            
            // Notify client that reset is complete
            if (pSh2->eventCallback) {
                sh2_AsyncEvent_t event;
                event.eventId = SH2_RESET;
                pSh2->eventCallback(pSh2->eventCookie, &event);
            }
            break;
        default:
            pSh2->execBadPayload++;
            break;
    }
}

// ------------------------------------------------------------------------
// Multi-Instance Public API

sh2_Instance_t* sh2_createInstance(void) {
    sh2_Instance_t* instance = (sh2_Instance_t*)malloc(sizeof(sh2_Instance_t));
    if (!instance) return NULL;
    
    memset(instance, 0, sizeof(sh2_Instance_t));
    instance->nextCmdSeq = 1;
    
    return instance;
}

void sh2_destroyInstance(sh2_Instance_t* pInstance) {
    if (pInstance) {
        if (pInstance->pShtp) {
            shtp_close(pInstance->pShtp);
        }
        free(pInstance);
    }
}

int sh2_openInstance(sh2_Instance_t* pInstance, sh2_Hal_t *pHal,
                     sh2_EventCallback_t *eventCallback, void *eventCookie) {
    if (!pInstance || !pHal) return SH2_ERR_BAD_PARAM;

    // Clear instance state
    memset(pInstance, 0, sizeof(sh2_Instance_t));
    pInstance->resetComplete = false;
    
    // Store references
    pInstance->pHal = pHal;
    pInstance->eventCallback = eventCallback;
    pInstance->eventCookie = eventCookie;
    pInstance->nextCmdSeq = 1;

    // Open SHTP layer
    pInstance->pShtp = shtp_open(pInstance->pHal);
    if (pInstance->pShtp == 0) {
        return SH2_ERR;
    }

    // Register SHTP handlers for this instance
    shtp_listenChan(pInstance->pShtp, CHAN_SENSORHUB_CONTROL, sensorhubControlHdlr, pInstance);
    shtp_listenChan(pInstance->pShtp, CHAN_SENSORHUB_INPUT, sensorhubInputNormalHdlr, pInstance);
    shtp_listenChan(pInstance->pShtp, CHAN_SENSORHUB_INPUT_WAKE, sensorhubInputWakeHdlr, pInstance);
    shtp_listenChan(pInstance->pShtp, CHAN_SENSORHUB_INPUT_GIRV, sensorhubInputGyroRvHdlr, pInstance);
    shtp_listenChan(pInstance->pShtp, CHAN_EXECUTABLE_DEVICE, executableDeviceHdlr, pInstance);

    // Wait for reset notifications
    uint32_t start_us = pInstance->pHal->getTimeUs(pInstance->pHal);
    uint32_t now_us = start_us;
    while (((now_us - start_us) < ADVERT_TIMEOUT_US) && (!pInstance->resetComplete)) {
        shtp_service(pInstance->pShtp);
        now_us = pInstance->pHal->getTimeUs(pInstance->pHal);
    }
    
    return SH2_OK;
}

void sh2_closeInstance(sh2_Instance_t* pInstance) {
    if (pInstance && pInstance->pShtp) {
        shtp_close(pInstance->pShtp);
        pInstance->pShtp = NULL;
    }
}

void sh2_serviceInstance(sh2_Instance_t* pInstance) {
    if (pInstance && pInstance->pShtp) {
        shtp_service(pInstance->pShtp);
    }
}

int sh2_setSensorCallbackInstance(sh2_Instance_t* pInstance, 
                                  sh2_SensorCallback_t *callback, void *cookie) {
    if (!pInstance) return SH2_ERR_BAD_PARAM;
    
    pInstance->sensorCallback = callback;
    pInstance->sensorCookie = cookie;
    return SH2_OK;
}

int sh2_setSensorConfigInstance(sh2_Instance_t* pInstance, 
                                sh2_SensorId_t sensorId, const sh2_SensorConfig_t *pConfig) {
    if (!pInstance || !pConfig) return SH2_ERR_BAD_PARAM;
    
    SetFeatureReport_t req;
    uint8_t flags = 0;
    
    if (pConfig->changeSensitivityEnabled)  flags |= FEAT_CHANGE_SENSITIVITY_ENABLED;
    if (pConfig->changeSensitivityRelative) flags |= FEAT_CHANGE_SENSITIVITY_RELATIVE;
    if (pConfig->wakeupEnabled)             flags |= FEAT_WAKE_ENABLED;
    if (pConfig->alwaysOnEnabled)           flags |= FEAT_ALWAYS_ON_ENABLED;
    if (pConfig->sniffEnabled)              flags |= FEAT_SNIFF_ENABLED;

    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_SET_FEATURE_CMD;
    req.featureReportId = sensorId;
    req.flags = flags;
    req.changeSensitivity = pConfig->changeSensitivity;
    req.reportInterval_uS = pConfig->reportInterval_us;
    req.batchInterval_uS = pConfig->batchInterval_us;
    req.sensorSpecific = pConfig->sensorSpecific;

    return sendCtrl(pInstance, (uint8_t *)&req, sizeof(req));
}

int sh2_getProdIdsInstance(sh2_Instance_t* pInstance, sh2_ProductIds_t *prodIds) {
    if (!pInstance || !prodIds) return SH2_ERR_BAD_PARAM;
    
    ProdIdReq_t req;
    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_PROD_ID_REQ;
    
    // This is a simplified implementation
    // Full implementation would handle the async response
    return sendCtrl(pInstance, (uint8_t *)&req, sizeof(req));
}

// ------------------------------------------------------------------------
// Legacy Global API (for backward compatibility)

int sh2_open(sh2_Hal_t *pHal, sh2_EventCallback_t *eventCallback, void *eventCookie) {
    if (!default_instance) {
        default_instance = sh2_createInstance();
        if (!default_instance) return SH2_ERR;
    }
    return sh2_openInstance(default_instance, pHal, eventCallback, eventCookie);
}

void sh2_close(void) {
    if (default_instance) {
        sh2_closeInstance(default_instance);
    }
}

void sh2_service(void) {
    if (default_instance) {
        sh2_serviceInstance(default_instance);
    }
}

int sh2_setSensorCallback(sh2_SensorCallback_t *callback, void *cookie) {
    if (!default_instance) return SH2_ERR;
    return sh2_setSensorCallbackInstance(default_instance, callback, cookie);
}

int sh2_setSensorConfig(sh2_SensorId_t sensorId, const sh2_SensorConfig_t *pConfig) {
    if (!default_instance) return SH2_ERR;
    return sh2_setSensorConfigInstance(default_instance, sensorId, pConfig);
}

int sh2_getProdIds(sh2_ProductIds_t *prodIds) {
    if (!default_instance) return SH2_ERR;
    return sh2_getProdIdsInstance(default_instance, prodIds);
}

// Stub implementations for other legacy functions
int sh2_devReset(void) { return SH2_ERR; }
int sh2_devOn(void) { return SH2_ERR; }
int sh2_devSleep(void) { return SH2_ERR; }
int sh2_getSensorConfig(sh2_SensorId_t sensorId, sh2_SensorConfig_t *pConfig) { 
    (void)sensorId; (void)pConfig; return SH2_ERR; 
}
int sh2_getMetadata(sh2_SensorId_t sensorId, sh2_SensorMetadata_t *pData) { 
    (void)sensorId; (void)pData; return SH2_ERR; 
}