#pragma once
// Vehicle Dynamics Engine - Telemetry System

#include "core/math/math_base.h"
#include "core/math/vec3.h"

typedef struct Vehicle Vehicle;

//-------------------------
// Types
//-------------------------

typedef struct Telemetry Telemetry;

// Telemetry data point
typedef struct TelemetryData {
    vde_real time;               // Simulation time (s)
    vde_vec3 position;           // Vehicle position
    vde_vec3 velocity;           // Vehicle velocity
    vde_vec3 acceleration;       // Vehicle acceleration
    vde_real steering_angle;     // Steering angle (rad)
    vde_real throttle;           // Throttle input [0,1]
    vde_real brake;              // Brake input [0,1]
    vde_real speed;              // Speed magnitude (m/s)
} TelemetryData;

// Telemetry callback function
typedef void (*TelemetryCallback)(const TelemetryData* data, void* user_data);

//-------------------------
// API Functions
//-------------------------

VDE_API Telemetry* telemetry_create(void);
VDE_API void telemetry_destroy(Telemetry* telem);

// Register callback
VDE_API void telemetry_register_callback(
    Telemetry* telem,
    TelemetryCallback callback,
    void* user_data
);

// Record data
VDE_API void telemetry_record(
    Telemetry* telem,
    vde_real time,
    const Vehicle* vehicle
);

// Export to file
VDE_API int telemetry_export_csv(
    const Telemetry* telem,
    const char* filename
);

// Clear recorded data
VDE_API void telemetry_clear(Telemetry* telem);