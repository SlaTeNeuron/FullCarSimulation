#include "simulation/telemetry.h"
#include "vehicle/vehicle.h"
#include <stdlib.h>
#include <stdio.h>

//-------------------------
// Internal State
//-------------------------

#define TELEMETRY_BUFFER_SIZE 10000

struct Telemetry {
    TelemetryData* buffer;       // Ring buffer of telemetry data
    int capacity;                // Buffer capacity
    int count;                   // Number of recorded samples
    int write_index;             // Current write position
    TelemetryCallback callback;  // Optional callback
    void* user_data;             // User data for callback
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create telemetry system
 * 
 * Output:
 *   - Returns pointer to Telemetry, or NULL on failure
 */
Telemetry* telemetry_create(void) {
    Telemetry* telem = (Telemetry*)malloc(sizeof(Telemetry));
    if (!telem) return NULL;
    
    telem->capacity = TELEMETRY_BUFFER_SIZE;
    telem->buffer = (TelemetryData*)malloc(telem->capacity * sizeof(TelemetryData));
    
    if (!telem->buffer) {
        free(telem);
        return NULL;
    }
    
    telem->count = 0;
    telem->write_index = 0;
    telem->callback = NULL;
    telem->user_data = NULL;
    
    return telem;
}

/**
 * Destroy telemetry system
 * 
 * Input:
 *   - telem: Telemetry to destroy (can be NULL)
 */
void telemetry_destroy(Telemetry* telem) {
    if (!telem) return;
    
    if (telem->buffer) {
        free(telem->buffer);
    }
    free(telem);
}

//-------------------------
// Callback
//-------------------------

/**
 * Register telemetry callback
 * 
 * Callback is called every time telemetry data is recorded.
 * 
 * Input:
 *   - telem: Telemetry system (must be non-NULL)
 *   - callback: Callback function (can be NULL to unregister)
 *   - user_data: User data passed to callback (can be NULL)
 */
void telemetry_register_callback(
    Telemetry* telem,
    TelemetryCallback callback,
    void* user_data
) {
    if (!telem) return;
    
    telem->callback = callback;
    telem->user_data = user_data;
}

//-------------------------
// Recording
//-------------------------

/**
 * Record telemetry data from vehicle
 * 
 * Input:
 *   - telem: Telemetry system (must be non-NULL)
 *   - time: Current simulation time
 *   - vehicle: Vehicle to record data from (must be non-NULL)
 * 
 * Functionality:
 *   1. Extract relevant data from vehicle
 *   2. Store in ring buffer
 *   3. Call registered callback if present
 */
void telemetry_record(
    Telemetry* telem,
    vde_real time,
    const Vehicle* vehicle
) {
    if (!telem || !vehicle) return;
    
    // TODO: Extract data from vehicle
    TelemetryData data;
    data.time = time;
    data.position = vde_vec3_zero();      // TODO: Get from vehicle
    data.velocity = vde_vec3_zero();      // TODO: Get from vehicle
    data.acceleration = vde_vec3_zero();  // TODO: Get from vehicle
    data.steering_angle = (vde_real)0.0;  // TODO: Get from vehicle
    data.throttle = (vde_real)0.0;        // TODO: Get from vehicle
    data.brake = (vde_real)0.0;           // TODO: Get from vehicle
    data.speed = (vde_real)0.0;           // TODO: Compute from velocity
    
    // Store in ring buffer
    telem->buffer[telem->write_index] = data;
    telem->write_index = (telem->write_index + 1) % telem->capacity;
    if (telem->count < telem->capacity) {
        telem->count++;
    }
    
    // Call callback if registered
    if (telem->callback) {
        telem->callback(&data, telem->user_data);
    }
}

//-------------------------
// Export
//-------------------------

/**
 * Export telemetry data to CSV file
 * 
 * Input:
 *   - telem: Telemetry system (must be non-NULL)
 *   - filename: Output filename (must be non-NULL)
 * 
 * Output:
 *   - Returns 0 on success, -1 on failure
 * 
 * Functionality:
 *   Writes all recorded telemetry data to CSV file.
 */
int telemetry_export_csv(
    const Telemetry* telem,
    const char* filename
) {
    if (!telem || !filename) return -1;
    
    FILE* file = fopen(filename, "w");
    if (!file) return -1;
    
    // Write CSV header
    fprintf(file, "time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,");
    fprintf(file, "acc_x,acc_y,acc_z,steering,throttle,brake,speed\n");
    
    // TODO: Write data rows
    // For each telemetry sample in buffer
    
    fclose(file);
    return 0;
}

/**
 * Clear all recorded telemetry data
 * 
 * Input:
 *   - telem: Telemetry system (must be non-NULL)
 */
void telemetry_clear(Telemetry* telem) {
    if (!telem) return;
    
    telem->count = 0;
    telem->write_index = 0;
}
