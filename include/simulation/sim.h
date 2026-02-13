#pragma once
// Vehicle Dynamics Engine - Simulation Core

#include <stdint.h>
#include "math/math_base.h"

//-------------------------
// Export / API Configuration
//-------------------------

/* Export macro for Windows DLL */
#ifdef _WIN32
    #ifdef BUILDING_DLL
        #define SIM_API __declspec(dllexport)
    #elif defined(STATIC_BUILD)
        #define SIM_API
    #else
        #define SIM_API __declspec(dllimport)
    #endif
#else
    #define SIM_API
#endif

//-------------------------
// Types
//-------------------------

typedef struct Sim Sim;

/* Simulation state for Unity interface */
typedef struct SimState {
    vde_real time;
    vde_real px, py, pz;
    vde_real yaw, pitch, roll;
    vde_real vel, accel;
    vde_real steeringAngle;
    vde_real motorRPM;
} SimState;

/* Telemetry callback: pointer to array of doubles, length */
typedef void (*telemetry_cb_t)(const vde_real* data, int len, void* user);

//-------------------------
// API Functions
//-------------------------

#ifdef __cplusplus
extern "C" {
#endif

/* Create/destroy simulation */
SIM_API Sim* sim_create(vde_real timestep);
SIM_API void sim_destroy(Sim* s);

/* Control simulation lifecycle */
SIM_API int sim_start(Sim* s);    /* returns 0 on success */
SIM_API int sim_stop(Sim* s);
SIM_API int sim_step(Sim* s);     /* advances one internal step */

/* Driver inputs (throttle, brake, steer) each in [-1,1] */
SIM_API void sim_set_inputs(Sim* s, vde_real throttle, vde_real brake, vde_real steer);

/* Register telemetry callback (C-thread or main thread — implementation currently calls from sim_step) */
SIM_API void sim_register_telemetry_cb(Sim* s, telemetry_cb_t cb, void* user);

/* Example convenience: get latest telemetry snapshot (copy into buffer provided) */
SIM_API int sim_get_latest_telemetry(Sim* s, vde_real* out_buffer, int max_len);

/* Get complete simulation state (for Unity interface) */
SIM_API void sim_get_state(Sim* s, SimState* out);

#ifdef __cplusplus
}
#endif