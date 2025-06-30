/* sh2_multi.h - Multi-instance wrapper for SH2 library */
#ifndef SH2_MULTI_H
#define SH2_MULTI_H

#include "sh2.h"
#include "sh2_err.h"     // Add this for error constants
#include "sh2_SensorValue.h"

#ifdef __cplusplus
extern "C" {
#endif

// Maximum number of SH2 instances supported
#define SH2_MAX_INSTANCES 4

// SH2 instance state structure
typedef struct {
    bool active;
    bool initialized;
    sh2_Hal_t *hal;
    sh2_EventCallback_t *eventCallback;
    void *eventCookie;
    sh2_SensorCallback_t *sensorCallback;
    void *sensorCookie;
    
    // Store the actual SH2 state when this instance is not active
    uint8_t sh2_state_backup[1024];  // Adjust size as needed
    bool has_backup;
} sh2_instance_t;

// Multi-instance SH2 API
int sh2_multi_open(int instance_id, sh2_Hal_t *pHal, 
                   sh2_EventCallback_t *eventCallback, void *eventCookie);
void sh2_multi_close(int instance_id);
void sh2_multi_service(int instance_id);
int sh2_multi_setSensorCallback(int instance_id, sh2_SensorCallback_t *callback, void *cookie);
int sh2_multi_setSensorConfig(int instance_id, sh2_SensorId_t sensorId, const sh2_SensorConfig_t *pConfig);

// Internal functions
void sh2_multi_set_active(int instance_id);
void sh2_multi_backup_state(int instance_id);
void sh2_multi_restore_state(int instance_id);

#ifdef __cplusplus
}
#endif

#endif // SH2_MULTI_H