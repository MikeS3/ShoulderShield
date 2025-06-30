/* sh2_multi.c - Multi-instance wrapper for SH2 library */

#include "sh2_multi.h"
#include "sh2_err.h"   // Add this for SH2_OK, SH2_ERR, etc.
#include <string.h>
#include <stdio.h>

// Global instance management
static sh2_instance_t instances[SH2_MAX_INSTANCES];
static int current_active_instance = -1;
static bool multi_initialized = false;

// Initialize the multi-instance system
static void sh2_multi_init(void) {
    if (multi_initialized) return;
    
    for (int i = 0; i < SH2_MAX_INSTANCES; i++) {
        memset(&instances[i], 0, sizeof(sh2_instance_t));
        instances[i].active = false;
        instances[i].initialized = false;
    }
    current_active_instance = -1;
    multi_initialized = true;
}

// Set an instance as active in the global SH2 state
void sh2_multi_set_active(int instance_id) {
    if (instance_id < 0 || instance_id >= SH2_MAX_INSTANCES) return;
    if (!instances[instance_id].initialized) return;
    if (current_active_instance == instance_id) return; // Already active
    
    sh2_instance_t *inst = &instances[instance_id];
    
    // Close current SH2 session if any
    if (current_active_instance >= 0) {
        sh2_close();
    }
    
    // Open SH2 with this instance's HAL
    int result = sh2_open(inst->hal, inst->eventCallback, inst->eventCookie);
    if (result == SH2_OK) {
        // Restore sensor callback
        if (inst->sensorCallback) {
            sh2_setSensorCallback(inst->sensorCallback, inst->sensorCookie);
        }
        current_active_instance = instance_id;
        printf("SH2 switched to instance %d\n", instance_id);
    } else {
        printf("Failed to switch to SH2 instance %d\n", instance_id);
    }
}

// Open a new SH2 instance
int sh2_multi_open(int instance_id, sh2_Hal_t *pHal, 
                   sh2_EventCallback_t *eventCallback, void *eventCookie) {
    if (!multi_initialized) sh2_multi_init();
    
    if (instance_id < 0 || instance_id >= SH2_MAX_INSTANCES) {
        return SH2_ERR_BAD_PARAM;
    }
    
    sh2_instance_t *inst = &instances[instance_id];
    
    // Store instance parameters
    inst->hal = pHal;
    inst->eventCallback = eventCallback;
    inst->eventCookie = eventCookie;
    inst->sensorCallback = NULL;
    inst->sensorCookie = NULL;
    inst->initialized = true;
    
    printf("SH2 instance %d registered\n", instance_id);
    
    // If this is the first instance, make it active
    if (current_active_instance == -1) {
        sh2_multi_set_active(instance_id);
    }
    
    return SH2_OK;
}

// Close an SH2 instance
void sh2_multi_close(int instance_id) {
    if (instance_id < 0 || instance_id >= SH2_MAX_INSTANCES) return;
    
    sh2_instance_t *inst = &instances[instance_id];
    if (!inst->initialized) return;
    
    // If this is the currently active instance, close SH2
    if (current_active_instance == instance_id) {
        sh2_close();
        current_active_instance = -1;
    }
    
    // Clear instance data
    inst->initialized = false;
    inst->active = false;
    
    printf("SH2 instance %d closed\n", instance_id);
}

// Service an SH2 instance
void sh2_multi_service(int instance_id) {
    if (instance_id < 0 || instance_id >= SH2_MAX_INSTANCES) return;
    if (!instances[instance_id].initialized) return;
    
    // Switch to this instance if not already active
    sh2_multi_set_active(instance_id);
    
    // Service the SH2 library
    sh2_service();
}

// Set sensor callback for an instance
int sh2_multi_setSensorCallback(int instance_id, sh2_SensorCallback_t *callback, void *cookie) {
    if (instance_id < 0 || instance_id >= SH2_MAX_INSTANCES) {
        return SH2_ERR_BAD_PARAM;
    }
    if (!instances[instance_id].initialized) {
        return SH2_ERR;
    }
    
    sh2_instance_t *inst = &instances[instance_id];
    inst->sensorCallback = callback;
    inst->sensorCookie = cookie;
    
    // If this instance is currently active, set the callback immediately
    if (current_active_instance == instance_id) {
        return sh2_setSensorCallback(callback, cookie);
    }
    
    return SH2_OK;
}

// Configure sensor for an instance
int sh2_multi_setSensorConfig(int instance_id, sh2_SensorId_t sensorId, const sh2_SensorConfig_t *pConfig) {
    if (instance_id < 0 || instance_id >= SH2_MAX_INSTANCES) {
        return SH2_ERR_BAD_PARAM;
    }
    if (!instances[instance_id].initialized) {
        return SH2_ERR;
    }
    
    // Switch to this instance
    sh2_multi_set_active(instance_id);
    
    // Configure the sensor
    int result = sh2_setSensorConfig(sensorId, pConfig);
    
    printf("Instance %d sensor %d config: %s\n", instance_id, sensorId, 
           (result == SH2_OK) ? "OK" : "FAILED");
    
    return result;
}

// Backup and restore functions (placeholders for future enhancement)
void sh2_multi_backup_state(int instance_id) {
    // TODO: Implement state backup if needed
    (void)instance_id;
}

void sh2_multi_restore_state(int instance_id) {
    // TODO: Implement state restore if needed  
    (void)instance_id;
}