// Vehicle Dynamics Engine - Unity API Implementation
// Implements the DLL boundary functions declared in unity_api.h

// Ensure we're exporting functions, not importing
#ifndef BUILDING_DLL
#define BUILDING_DLL
#endif

#include "unity_api.h"
#include "vehicle/vehicle_model.h"
#include "vehicle/vehicle.h"
#include "simulation/simulation_loop.h"
#include "core/utils/logger.h"
#include "core/math/vec3.h"
#include "core/math/quat.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

//-------------------------
// Internal Simulation State
//-------------------------

// Error / log ring buffer parameters
#define SIM_ERROR_LOG_SLOTS   32    // Max entries retained (oldest dropped when full)
#define SIM_ERROR_LOG_MSG_LEN 256   // Max length of a single log message

typedef struct SimErrorEntry {
    double        sim_time;                    // Vehicle sim time at which entry was logged
    SimErrorLevel level;                       // Severity
    char          message[SIM_ERROR_LOG_MSG_LEN]; // Formatted text
} SimErrorEntry;

struct VehicleSimulation {
    Vehicle* vehicle;           // Vehicle model
    double timestep;            // Fixed physics timestep
    double current_time;        // Current simulation time
    DriverInputs inputs;        // Current driver inputs
    int initialized;            // Initialization flag

    // Telemetry recording
    TelemetryCallback telemetry_callback;  // Callback function
    void* telemetry_user_data;             // User data for callback
    int telemetry_recording;               // Recording flag
    int telemetry_frame_count;             // Number of recorded frames

    // Performance stats
    int total_steps;                       // Total steps executed
    double total_step_time;                // Cumulative step time
    double max_step_time;                  // Maximum step time

    //--- Error / diagnostic system ---
    char last_error[512];                  // Most recent ERROR-level message (empty = none)
    int  has_error;                        // 1 if an error has been set since last ClearError

    // Ring buffer: stores up to SIM_ERROR_LOG_SLOTS recent entries (all severity levels)
    SimErrorEntry error_log[SIM_ERROR_LOG_SLOTS];
    int error_log_write_pos;               // Next write slot (0..SLOTS-1, wraps)
    int error_log_total;                   // Total entries ever written (monotonic)

    // Rate-limiting for "Step skipped: not initialized" diagnostic
    int step_skip_count;                   // How many Step calls were skipped

    // Real-time log callback registered by Unity
    SimLogCallback log_callback;           // NULL if not registered
    void* log_callback_user_data;          // Passed back verbatim to the callback
};

//-------------------------
// Internal Logging Helpers
//-------------------------

static const char* sim_level_name(SimErrorLevel level) {
    switch (level) {
        case SIM_LOG_DEBUG: return "DEBUG";
        case SIM_LOG_INFO:  return "INFO ";
        case SIM_LOG_WARN:  return "WARN ";
        case SIM_LOG_ERROR: return "ERROR";
        default:            return "?    ";
    }
}

// Core internal logging function - writes to the ring buffer, last_error, and the
// Unity callback.  Never call this with a NULL sim.
static void sim_record_log(VehicleSimulation* sim, SimErrorLevel level, const char* fmt, ...) {
    char msg[SIM_ERROR_LOG_MSG_LEN];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);
    msg[sizeof(msg) - 1] = '\0';

    // Write to ring buffer (all severity levels)
    int slot = sim->error_log_write_pos;
    sim->error_log[slot].sim_time = sim->current_time;
    sim->error_log[slot].level    = level;
    strncpy(sim->error_log[slot].message, msg, SIM_ERROR_LOG_MSG_LEN - 1);
    sim->error_log[slot].message[SIM_ERROR_LOG_MSG_LEN - 1] = '\0';
    sim->error_log_write_pos = (sim->error_log_write_pos + 1) % SIM_ERROR_LOG_SLOTS;
    sim->error_log_total++;

    // Keep last_error for ERROR level
    if (level >= SIM_LOG_ERROR) {
        snprintf(sim->last_error, sizeof(sim->last_error),
                 "[T=%.3f] %s", sim->current_time, msg);
        sim->last_error[sizeof(sim->last_error) - 1] = '\0';
        sim->has_error = 1;
    }

    // Fire the Unity real-time callback
    if (sim->log_callback) {
        char full[SIM_ERROR_LOG_MSG_LEN + 64];
        snprintf(full, sizeof(full), "[T=%.3f][%s] %s",
                 sim->current_time, sim_level_name(level), msg);
        full[sizeof(full) - 1] = '\0';
        sim->log_callback(level, full, sim->log_callback_user_data);
    }

    // Mirror to the VDE logger (writes to stdout / log file)
    if (level >= SIM_LOG_ERROR) {
        VDE_LOG_ERROR("[UnityAPI] %s", msg);
    } else if (level == SIM_LOG_WARN) {
        VDE_LOG_WARN("[UnityAPI] %s", msg);
    } else {
        VDE_LOG_INFO("[UnityAPI] %s", msg);
    }
}

// Convenience macros (sim_ptr may be NULL for the ERROR variant - checked inside)
#define SIM_LOG_MSG(sim_ptr, lvl, fmt, ...)  do { if(sim_ptr) sim_record_log((sim_ptr),(lvl),(fmt),##__VA_ARGS__); } while(0)
#define SIM_ERROR(sim_ptr, fmt, ...)  SIM_LOG_MSG(sim_ptr, SIM_LOG_ERROR, fmt, ##__VA_ARGS__)
#define SIM_WARN(sim_ptr,  fmt, ...)  SIM_LOG_MSG(sim_ptr, SIM_LOG_WARN,  fmt, ##__VA_ARGS__)
#define SIM_INFO(sim_ptr,  fmt, ...)  SIM_LOG_MSG(sim_ptr, SIM_LOG_INFO,  fmt, ##__VA_ARGS__)

//-------------------------
// Coordinate System Conversion
//-------------------------
//
// Internal physics convention (right-handed, Y-down):
//   X = forward (longitudinal)
//   Y = down    (vertical, negative = up)
//   Z = right   (lateral)
//
// Unity convention (left-handed, Y-up):
//   X = right   (lateral)
//   Y = up      (vertical)
//   Z = forward (longitudinal)
//
// Transformation matrix T (physics → Unity):
//   T = [[ 0,  0, 1 ],
//        [ 0, -1, 0 ],   ← Y is negated (down → up)
//        [ 1,  0, 0 ]]
//   det(T) = +1  →  proper rotation.
//   Quaternions additionally require conjugation to correct the rotation sense when
//   transforming into Unity's left-handed system.
//
// Rules applied at the DLL boundary (Physics → Unity):
//   Position / linear velocity : Unity(x,y,z) = ( Physics.z, -Physics.y,  Physics.x)
//   Angular velocity           : Unity(x,y,z) = ( Physics.z, -Physics.y,  Physics.x)
//                                (same axes as position — det=1, no extra sign flip)
//   Quaternion                 : Unity(w,x,y,z) = (Physics.w, -Physics.z, Physics.y, -Physics.x)
//                                Axis remapping PLUS conjugate to flip rotation sense for LH system.
//                                The Y-up/Y-down negation (−py) and the LH conjugate negation cancel,
//                                yielding +py. X and Z acquire the conjugate minus sign.
//   Scalar yaw rate            : Unity_yaw_rate = -Physics.angular_velocity.y
//-------------------------

static inline void phys_to_unity_pos(double px, double py, double pz,
                                      double* ux, double* uy, double* uz)
{
    *ux =  pz;  // Unity right   = Physics Z (right)
    *uy = -py;  // Unity up      = -Physics Y (Y is down in physics)
    *uz =  px;  // Unity forward = Physics X (forward)
}

static inline void phys_to_unity_angular(double px, double py, double pz,
                                          double* ux, double* uy, double* uz)
{
    // det(T) = +1, so angular (pseudo) vectors transform identically to position vectors.
    *ux =  pz;  // rotation rate about Unity.X =  Physics angular.Z
    *uy = -py;  // rotation rate about Unity.Y = -Physics angular.Y (down → up, sign flips)
    *uz =  px;  // rotation rate about Unity.Z =  Physics angular.X
}

static inline void phys_to_unity_quat(double pw, double px, double py, double pz,
                                       double* uw, double* ux, double* uy, double* uz)
{
    // Step 1 – remap axes: (w, x, y, z) → (w, pz, -py, px)  [same rule as positions]
    // Step 2 – conjugate (negate imaginary part) to flip rotation sense for LH Unity system.
    // Combined: Unity(w,x,y,z) = (Physics.w, -Physics.z, Physics.y, -Physics.x)
    // Note: the Y-down→Y-up negation (step 1: -py) and the LH conjugate negation (step 2)
    //       cancel on the Y component, leaving it positive.
    *uw =  pw;
    *ux = -pz;  // Unity.X component = -(Physics.Z)
    *uy =  py;  // Unity.Y component = +(Physics.Y)  [double-negation cancels]
    *uz = -px;  // Unity.Z component = -(Physics.X)
}

//=========================
// 1. LIFECYCLE MANAGEMENT
//=========================

UNITY_API VehicleSimulation* VehicleSim_Create(double timestep) {
    if (timestep <= 0.0) {
        VDE_LOG_ERROR("Invalid timestep: %f", timestep);
        return NULL;
    }
    
    VehicleSimulation* sim = (VehicleSimulation*)malloc(sizeof(VehicleSimulation));
    if (!sim) {
        VDE_LOG_ERROR("Failed to allocate simulation");
        return NULL;
    }
    
    // Initialize simulation state
    memset(sim, 0, sizeof(VehicleSimulation));
    sim->timestep           = timestep;
    sim->current_time       = 0.0;
    sim->initialized        = 0;
    sim->telemetry_callback = NULL;
    sim->telemetry_user_data = NULL;
    sim->telemetry_recording = 0;
    sim->telemetry_frame_count = 0;
    sim->total_steps        = 0;
    sim->total_step_time    = 0.0;
    sim->max_step_time      = 0.0;
    sim->has_error          = 0;
    sim->last_error[0]      = '\0';
    sim->error_log_write_pos = 0;
    sim->error_log_total    = 0;
    sim->step_skip_count    = 0;
    sim->log_callback       = NULL;
    sim->log_callback_user_data = NULL;

    // Create vehicle model
    sim->vehicle = vehicle_model_create();
    if (!sim->vehicle) {
        SIM_ERROR(sim, "vehicle_model_create() returned NULL - out of memory?");
        free(sim);
        return NULL;
    }

    SIM_INFO(sim, "Simulation created. timestep=%.6f s. Call LoadVehicle -> LoadTrack -> Initialize before Step.", timestep);
    return sim;
}

UNITY_API void VehicleSim_Destroy(VehicleSimulation* sim) {
    if (!sim) return;
    
    if (sim->vehicle) {
        vehicle_model_destroy(sim->vehicle);
    }
    
    free(sim);
    VDE_LOG_INFO("Simulation destroyed");
}

UNITY_API int VehicleSim_LoadVehicle(VehicleSimulation* sim, const char* vehicle_config_path) {
    if (!sim) {
        VDE_LOG_ERROR("LoadVehicle: null sim handle");
        return -1;
    }
    if (!vehicle_config_path) {
        SIM_ERROR(sim, "LoadVehicle: null config path");
        return -1;
    }
    if (!sim->vehicle) {
        SIM_ERROR(sim, "LoadVehicle: vehicle pointer is NULL - Create() may have failed");
        return -1;
    }

    // ALPHA: parameter file parsing not yet implemented; using hardcoded defaults
    SIM_WARN(sim, "LoadVehicle: path='%s' - ALPHA: using hardcoded TBRe defaults (file not parsed yet)",
             vehicle_config_path);
    int ok = vehicle_model_init_from_params(sim->vehicle, vehicle_config_path);
    if (!ok) {
        SIM_ERROR(sim, "LoadVehicle: vehicle_model_init_from_params returned failure");
        return -1;
    }
    SIM_INFO(sim, "LoadVehicle: OK (alpha defaults applied)");
    return 0;
}

UNITY_API int VehicleSim_LoadTrack(VehicleSimulation* sim, const char* track_config_path) {
    if (!sim) {
        VDE_LOG_ERROR("LoadTrack: null sim handle");
        return -1;
    }
    if (!track_config_path) {
        SIM_ERROR(sim, "LoadTrack: null config path");
        return -1;
    }
    // ALPHA: track loading not yet implemented
    SIM_WARN(sim, "LoadTrack: path='%s' - ALPHA: track loading not implemented, continuing",
             track_config_path);
    return 0;
}

UNITY_API int VehicleSim_Initialize(VehicleSimulation* sim) {
    if (!sim) {
        VDE_LOG_ERROR("Initialize: null sim handle");
        return -1;
    }
    if (!sim->vehicle) {
        SIM_ERROR(sim, "Initialize: vehicle pointer is NULL - Create() may have failed");
        return -1;
    }

    SIM_INFO(sim, "Initialize: running vehicle_model_validate...");
    if (!vehicle_model_validate(sim->vehicle)) {
        SIM_ERROR(sim,
            "Initialize: vehicle_model_validate() returned 0. "
            "The vehicle model is in an invalid state. "
            "Check that LoadVehicle succeeded and that no required parameters are zero.");
        return -1;
    }

    sim->initialized = 1;
    SIM_INFO(sim, "Initialize: SUCCESS. Simulation is ready. Call VehicleSim_Step to advance time.");
    return 0;
}

UNITY_API void VehicleSim_Reset(VehicleSimulation* sim) {
    if (!sim) return;

    SIM_INFO(sim, "Reset called. Clearing simulation state.");

    // Reset simulation state
    sim->current_time = 0.0;
    sim->initialized  = 0;
    sim->step_skip_count = 0;  // Clear skip counter - fresh start after reset
    memset(&sim->inputs, 0, sizeof(DriverInputs));
    sim->telemetry_frame_count = 0;
    sim->total_steps = 0;
    sim->total_step_time = 0.0;
    sim->max_step_time = 0.0;

    // Reset vehicle to initial state
    if (sim->vehicle) {
        vehicle_destroy(sim->vehicle);
        sim->vehicle = vehicle_create();
        if (!sim->vehicle) {
            SIM_ERROR(sim,
                "Reset: vehicle_create() returned NULL after vehicle_destroy(). "
                "sim->initialized remains 0. Call LoadVehicle + Initialize again.");
            return;
        }
    }

    // Note: initialized is intentionally left as 0 after Reset.
    // Caller must call VehicleSim_Initialize() again before stepping.
    SIM_INFO(sim,
        "Reset complete. initialized=0 - call VehicleSim_Initialize() before next VehicleSim_Step().");
}

//=========================
// 2. SIMULATION CONTROL
//=========================

UNITY_API void VehicleSim_Step(VehicleSimulation* sim, const DriverInputs* inputs) {
    if (!sim) {
        // Can't log without a sim; the caller should have checked Create() != NULL
        VDE_LOG_ERROR("VehicleSim_Step: null sim handle - time will NOT advance");
        return;
    }

    if (!sim->initialized) {
        // Rate-limited diagnostic: log on 1st, 10th, 100th, 1000th, then every 10000th skip
        sim->step_skip_count++;
        int n = sim->step_skip_count;
        if (n == 1 || n == 10 || n == 100 || n == 1000 || (n % 10000 == 0)) {
            SIM_ERROR(sim,
                "VehicleSim_Step SKIPPED: initialized=%d (skip count: %d). "
                "Possible causes: (1) VehicleSim_Initialize() was never called, "
                "(2) Initialize() returned non-zero (check GetLastError / GetDiagnostics), "
                "(3) VehicleSim_Reset() was called and Initialize() not re-called. "
                "Sim time will NOT progress until initialized=1.",
                sim->initialized, n);
        }
        return;
    }

    // Validate inputs
    if (!inputs) {
        SIM_ERROR(sim, "VehicleSim_Step: null inputs pointer - step skipped");
        return;
    }
    
    // Store inputs for telemetry/debugging
    sim->inputs = *inputs;
    
    // Track step time for performance stats
    // ALPHA TODO: Use actual timing (clock_gettime or similar)
    double step_start_time = 0.0;  // Would use actual timing here
    
    // Apply inputs to vehicle model
    vehicle_set_inputs(sim->vehicle, 
                      (vde_real)inputs->throttle, 
                      (vde_real)inputs->brake, 
                      (vde_real)inputs->steering);
    
    // Execute one physics timestep
    vehicle_step(sim->vehicle, (vde_real)sim->timestep);
    sim->current_time += sim->timestep;
    
    // Update performance stats
    sim->total_steps++;
    double step_time = sim->timestep;  // ALPHA: placeholder, should be actual wall-clock time
    sim->total_step_time += step_time;
    if (step_time > sim->max_step_time) {
        sim->max_step_time = step_time;
    }
    
    // Handle telemetry recording
    if (sim->telemetry_recording) {
        sim->telemetry_frame_count++;
        // ALPHA TODO: Store telemetry frames to buffer
    }
    
    // Call telemetry callback if registered
    if (sim->telemetry_callback) {
        TelemetryFrame frame;
        VehicleSim_GetTelemetry(sim, &frame);
        sim->telemetry_callback(&frame, sim->telemetry_user_data);
    }
}

UNITY_API void VehicleSim_StepMultiple(VehicleSimulation* sim, const DriverInputs* inputs, int num_steps) {
    if (!sim) {
        VDE_LOG_ERROR("VehicleSim_StepMultiple: null sim handle");
        return;
    }
    if (!inputs) {
        SIM_ERROR(sim, "VehicleSim_StepMultiple: null inputs pointer");
        return;
    }
    if (num_steps <= 0) {
        SIM_WARN(sim, "VehicleSim_StepMultiple: num_steps=%d (must be > 0), nothing done", num_steps);
        return;
    }
    if (!sim->initialized) {
        // Delegate to VehicleSim_Step which has rate-limited error reporting
        VehicleSim_Step(sim, inputs);
        return;
    }

    for (int i = 0; i < num_steps; i++) {
        VehicleSim_Step(sim, inputs);
    }
}

UNITY_API double VehicleSim_GetTime(const VehicleSimulation* sim) {
    return sim ? sim->current_time : 0.0;
}

//=========================
// 3. INPUT UTILITIES
//=========================

UNITY_API void VehicleSim_GetInputs(const VehicleSimulation* sim, DriverInputs* out_inputs) {
    if (!sim || !out_inputs) return;
    
    *out_inputs = sim->inputs;
}

//=========================
// 4. OUTPUT INTERFACE
//=========================

UNITY_API void VehicleSim_GetRenderData(const VehicleSimulation* sim, VehicleRenderData* out_data) {
    if (!sim || !out_data) {
        if (sim && !out_data) SIM_ERROR((VehicleSimulation*)sim, "GetRenderData: null out_data pointer");
        return;
    }
    if (!sim->initialized) {
        SIM_WARN((VehicleSimulation*)sim,
            "GetRenderData called but sim not initialized - output will be zeroed");
    }
    
    // Initialize output
    memset(out_data, 0, sizeof(VehicleRenderData));
    out_data->simulation_time = sim->current_time;
    
    // Get vehicle state
    VehicleState state;
    vehicle_model_get_state(sim->vehicle, &state);
    
    // Fill chassis render data — all vectors converted to Unity space.
    // Physics: X=forward, Y=up, Z=right  →  Unity: X=right, Y=up, Z=forward
    phys_to_unity_pos(
        (double)state.position.x, (double)state.position.y, (double)state.position.z,
        &out_data->chassis.position_x, &out_data->chassis.position_y, &out_data->chassis.position_z);

    phys_to_unity_quat(
        (double)state.orientation.w,
        (double)state.orientation.x, (double)state.orientation.y, (double)state.orientation.z,
        &out_data->chassis.orientation_w,
        &out_data->chassis.orientation_x, &out_data->chassis.orientation_y, &out_data->chassis.orientation_z);

    phys_to_unity_pos(
        (double)state.velocity.x, (double)state.velocity.y, (double)state.velocity.z,
        &out_data->chassis.velocity_x, &out_data->chassis.velocity_y, &out_data->chassis.velocity_z);

    phys_to_unity_angular(
        (double)state.angular_velocity.x, (double)state.angular_velocity.y, (double)state.angular_velocity.z,
        &out_data->chassis.angular_vel_x, &out_data->chassis.angular_vel_y, &out_data->chassis.angular_vel_z);
    
    // Calculate speed in km/h
    vde_real speed_ms = vde_vec3_norm(&state.velocity);
    out_data->speed_kmh = (double)(speed_ms * 3.6);  // m/s to km/h
    
    // Current gear from inputs
    out_data->current_gear = sim->inputs.gear;
    
    // Steering wheel angle (approximate: steer input * max angle)
    out_data->steering_wheel_angle = sim->inputs.steering * 3.14159;  // ~180 degrees max
    
    // ALPHA: Wheel data - for now use simplified approach
    // TODO: Get actual wheel states when Vehicle structure is expanded
    for (int i = 0; i < 4; i++) {
        out_data->wheels[i].rotation_angle = (double)state.wheel_angular_vel[i] * sim->current_time;
        out_data->wheels[i].steer_angle = (i < 2) ? sim->inputs.steering * 0.5 : 0.0;  // Front wheels only
        out_data->wheels[i].is_grounded = 1;  // Assume grounded for now
        
        // Compute wheel positions in physics space (X=forward, Y=up, Z=right),
        // then convert to Unity space at the end.
        vde_real wheelbase = 1.531;  // m
        vde_real track = 1.2;       // m
        vde_real wheel_radius = 0.2;  // m
        
        // Physics-space offsets: X=longitudinal, Y=vertical, Z=lateral
        vde_real x_offset = ((i < 2) ? 0 : -wheelbase) - (vde_real)0.882;  // +X = forward
        vde_real z_offset = (i % 2 == 0) ? -track * 0.5 : track * 0.5;     // +Z = right

        vde_vec3 wheel_offset_local = vde_vec3_make(x_offset, -wheel_radius, z_offset);
        // Must use the conjugate of the physics orientation here.
        // phys_to_unity_quat() conjugates q_physics to correct rotation sense for Unity's
        // left-handed system.  vde_quat_rotate must apply the same conjugated rotation so
        // the wheel offsets orbit the chassis in the same direction Unity's quaternion rotates.
        vde_quat orientation_conj = vde_quat_conj(&state.orientation);
        vde_vec3 wheel_offset_world = vde_quat_rotate(&orientation_conj, &wheel_offset_local);
        vde_vec3 wheel_position;
        vde_vec3_add(&wheel_position, &state.position, &wheel_offset_world);
        
        // Convert physics-space wheel position to Unity space
        phys_to_unity_pos(
            (double)wheel_position.x, (double)wheel_position.y, (double)wheel_position.z,
            &out_data->wheels[i].position_x,
            &out_data->wheels[i].position_y,
            &out_data->wheels[i].position_z);

        out_data->wheels[i].suspension_deflection = (double)state.suspension_deflection[i];
    }
}

//=========================
// 5. TELEMETRY
//=========================

UNITY_API void VehicleSim_RegisterTelemetryCallback(
    VehicleSimulation* sim,
    TelemetryCallback callback,
    void* user_data
) {
    if (!sim) return;

    sim->telemetry_callback = callback;
    sim->telemetry_user_data = user_data;
    SIM_INFO(sim, "Telemetry callback %s.", callback ? "registered" : "cleared (NULL passed)");
}

UNITY_API void VehicleSim_GetTelemetry(const VehicleSimulation* sim, TelemetryFrame* out_telemetry) {
    if (!out_telemetry) return;
    if (!sim) {
        VDE_LOG_ERROR("GetTelemetry: null sim handle");
        return;
    }
    
    // Initialize output
    memset(out_telemetry, 0, sizeof(TelemetryFrame));
    
    // Time data
    out_telemetry->time = sim->current_time;
    out_telemetry->delta_time = sim->timestep;
    
    // Get vehicle state
    VehicleState state;
    vehicle_model_get_state(sim->vehicle, &state);
    
    // Chassis state — all vectors converted to Unity space.
    // Physics: X=forward, Y=up, Z=right  →  Unity: X=right, Y=up, Z=forward
    phys_to_unity_pos(
        (double)state.position.x, (double)state.position.y, (double)state.position.z,
        &out_telemetry->position[0], &out_telemetry->position[1], &out_telemetry->position[2]);

    phys_to_unity_pos(
        (double)state.velocity.x, (double)state.velocity.y, (double)state.velocity.z,
        &out_telemetry->velocity[0], &out_telemetry->velocity[1], &out_telemetry->velocity[2]);

    // Orientation quaternion — Unity (w,x,y,z) order
    {
        double uw, ux, uy, uz;
        phys_to_unity_quat(
            (double)state.orientation.w,
            (double)state.orientation.x, (double)state.orientation.y, (double)state.orientation.z,
            &uw, &ux, &uy, &uz);
        out_telemetry->orientation[0] = uw;
        out_telemetry->orientation[1] = ux;
        out_telemetry->orientation[2] = uy;
        out_telemetry->orientation[3] = uz;
    }

    phys_to_unity_angular(
        (double)state.angular_velocity.x, (double)state.angular_velocity.y, (double)state.angular_velocity.z,
        &out_telemetry->angular_velocity[0],
        &out_telemetry->angular_velocity[1],
        &out_telemetry->angular_velocity[2]);
    
    // Derived quantities
    vde_real speed_ms = vde_vec3_norm(&state.velocity);
    out_telemetry->speed = (double)speed_ms;
    // Yaw rate: rotation about the vertical (Y) axis in physics space.
    // The sign is negated because the transformation is improper (LH output).
    out_telemetry->yaw_rate = -(double)state.angular_velocity.y;
    
    // Driver inputs
    out_telemetry->throttle = sim->inputs.throttle;
    out_telemetry->brake = sim->inputs.brake;
    out_telemetry->steering = sim->inputs.steering;
    out_telemetry->gear = sim->inputs.gear;
    
    // Wheel angular velocities
    for (int i = 0; i < 4; i++) {
        out_telemetry->wheel_angular_vel[i] = (double)state.wheel_angular_vel[i];
        out_telemetry->suspension_deflection[i] = (double)state.suspension_deflection[i];
    }
    
    // ALPHA TODO: Add tire forces, load transfers, aero forces when available
}

UNITY_API void VehicleSim_StartTelemetryRecording(VehicleSimulation* sim) {
    if (!sim) return;

    sim->telemetry_recording = 1;
    sim->telemetry_frame_count = 0;
    SIM_INFO(sim, "Telemetry recording started.");
}

UNITY_API int VehicleSim_ExportTelemetryCSV(
    VehicleSimulation* sim,
    const char* output_path
) {
    if (!sim) {
        VDE_LOG_ERROR("ExportTelemetryCSV: null sim handle");
        return -1;
    }
    if (!output_path) {
        SIM_ERROR(sim, "ExportTelemetryCSV: null output_path");
        return -1;
    }
    // ALPHA: CSV export not yet implemented
    SIM_WARN(sim, "ExportTelemetryCSV: not implemented in alpha (path='%s')", output_path);
    return -1;
}

UNITY_API int VehicleSim_GetTelemetryFrameCount(const VehicleSimulation* sim) {
    return sim ? sim->telemetry_frame_count : 0;
}

UNITY_API void VehicleSim_ClearTelemetry(VehicleSimulation* sim) {
    if (!sim) return;

    sim->telemetry_frame_count = 0;
    sim->telemetry_recording   = 0;
    SIM_INFO(sim, "Telemetry cleared.");
}

//=========================
// 5. ASSET MANAGEMENT
//=========================

UNITY_API void VehicleSim_GetVehicleParameters(const VehicleSimulation* sim, VehicleParameters* out_params) {
    if (!sim || !out_params) return;
    
    // Return hardcoded TBRe-style vehicle parameters
    memset(out_params, 0, sizeof(VehicleParameters));
    
    // Mass properties (FSAE-style electric vehicle)
    out_params->mass = 250.0;  // kg (electric FSAE car)
    out_params->wheelbase = 1.55;  // m
    out_params->track_front = 1.2;  // m
    out_params->track_rear = 1.2;  // m
    out_params->cg_height = 0.28;  // m (low center of gravity)
    out_params->cg_to_front = 0.75;  // m
    out_params->cg_to_rear = 0.80;  // m
    
    // Inertia (estimated for FSAE vehicle)
    out_params->inertia_roll = 35.0;   // kg·m²
    out_params->inertia_pitch = 70.0;  // kg·m²
    out_params->inertia_yaw = 75.0;    // kg·m²
    
    // Tire properties (10-inch racing tires)
    out_params->tire_radius = 0.2286;  // m (9 inch diameter)
    out_params->tire_friction_coeff = 1.4;  // Racing slicks
    
    // Suspension (stiff racing setup)
    out_params->spring_rate_front = 35000.0;  // N/m
    out_params->spring_rate_rear = 35000.0;   // N/m
    out_params->damping_front = 2500.0;       // N·s/m
    out_params->damping_rear = 2500.0;        // N·s/m
    
    // Aerodynamics (basic undertray + small wings)
    out_params->drag_coefficient = 0.9;
    out_params->lift_coefficient = -1.2;  // Downforce
    out_params->frontal_area = 1.2;  // m²
    
    // Powertrain (electric motor)
    out_params->max_engine_torque = 180.0;  // Nm (at wheel)
    out_params->max_rpm = 20000.0;  // Electric motor high RPM
}

UNITY_API void VehicleSim_GetTrackInfo(const VehicleSimulation* sim, TrackInfo* out_info) {
    if (!sim || !out_info) return;
    
    // ALPHA: Return placeholder info
    memset(out_info, 0, sizeof(TrackInfo));
    strncpy(out_info->name, "Alpha Track", sizeof(out_info->name) - 1);
    out_info->length = 100.0;
    out_info->num_sectors = 1;
}

UNITY_API void VehicleSim_QuerySurface(const VehicleSimulation* sim, SurfaceQuery* query) {
    if (!sim || !query) return;
    
    // ALPHA: Return flat ground with standard friction.
    // Input position and output normal are expressed in Unity world space
    // (X=right, Y=up, Z=forward).  When a real track is implemented, convert
    // the input position to physics space (X=forward, Y=up, Z=right) before
    // querying, then convert the result normal back using phys_to_unity_pos().
    query->normal[0] = 0.0;
    query->normal[1] = 1.0;  // +Y up in Unity space
    query->normal[2] = 0.0;
    query->friction_coefficient = 1.0;
    query->elevation = 0.0;
    query->is_valid = 1;
}

//=========================
// 6. ERROR HANDLING
//=========================

UNITY_API void VehicleSim_SetLogCallback(
    VehicleSimulation* sim,
    SimLogCallback callback,
    void* user_data
) {
    if (!sim) return;
    sim->log_callback = callback;
    sim->log_callback_user_data = user_data;
    if (callback) {
        // Fire an immediate INFO message so the caller can confirm the callback fires
        SIM_INFO(sim,
            "Log callback registered. Severity: 0=DEBUG 1=INFO 2=WARN 3=ERROR. "
            "initialized=%d timestep=%.6f total_steps=%d current_time=%.6f",
            sim->initialized, sim->timestep, sim->total_steps, sim->current_time);
    }
}

UNITY_API const char* VehicleSim_GetLastError(const VehicleSimulation* sim) {
    if (!sim) return "VehicleSim_GetLastError: null sim handle";
    return sim->has_error ? sim->last_error : "(no error recorded)";
}

UNITY_API void VehicleSim_ClearError(VehicleSimulation* sim) {
    if (!sim) return;
    sim->last_error[0] = '\0';
    sim->has_error = 0;
    // Intentionally does NOT clear the ring-buffer log so post-mortem analysis is preserved
}

UNITY_API void VehicleSim_GetErrorLog(
    const VehicleSimulation* sim,
    char* out_buf,
    int buf_size
) {
    if (!out_buf || buf_size <= 0) return;
    out_buf[0] = '\0';

    if (!sim) {
        strncpy(out_buf, "(null sim)\n", (size_t)(buf_size - 1));
        return;
    }

    int total = sim->error_log_total;
    int n     = total < SIM_ERROR_LOG_SLOTS ? total : SIM_ERROR_LOG_SLOTS;

    if (n == 0) {
        strncpy(out_buf, "(no log entries)\n", (size_t)(buf_size - 1));
        return;
    }

    // Oldest entry: when buffer is full it's at write_pos; otherwise at 0
    int start = (total >= SIM_ERROR_LOG_SLOTS) ? sim->error_log_write_pos : 0;

    int written = 0;
    for (int i = 0; i < n && written < buf_size - 1; i++) {
        int slot = (start + i) % SIM_ERROR_LOG_SLOTS;
        const SimErrorEntry* e = &sim->error_log[slot];
        int lv = (int)e->level;
        if (lv < 0) lv = 0;
        if (lv > 3) lv = 3;
        int r = snprintf(out_buf + written, (size_t)(buf_size - written),
                         "[T=%8.3f][%s] %s\n",
                         e->sim_time, sim_level_name(lv), e->message);
        if (r > 0) written += r;
    }
    out_buf[buf_size - 1] = '\0';
}

UNITY_API void VehicleSim_GetDiagnostics(
    const VehicleSimulation* sim,
    char* out_buf,
    int buf_size
) {
    if (!out_buf || buf_size <= 0) return;
    out_buf[0] = '\0';

    if (!sim) {
        strncpy(out_buf, "VehicleSim_GetDiagnostics: null sim handle\n", (size_t)(buf_size - 1));
        return;
    }

    int w = 0;
    w += snprintf(out_buf + w, (size_t)(buf_size - w),
        "=== VehicleSim Diagnostics ===\n"
        "  initialized     : %d  <- must be 1 for Step to advance time\n"
        "  timestep        : %.6f s\n"
        "  current_time    : %.6f s\n"
        "  total_steps     : %d\n"
        "  step_skip_count : %d  <- Step calls skipped due to !initialized\n"
        "  vehicle ptr     : %s\n"
        "  has_error       : %d\n"
        "  error_log_total : %d entries (up to last %d retained)\n"
        "  last_error      : %s\n",
        sim->initialized,
        sim->timestep,
        sim->current_time,
        sim->total_steps,
        sim->step_skip_count,
        sim->vehicle ? "valid" : "NULL (!)",
        sim->has_error,
        sim->error_log_total, SIM_ERROR_LOG_SLOTS,
        sim->has_error ? sim->last_error : "(none)");

    if (w < buf_size - 1) {
        w += snprintf(out_buf + w, (size_t)(buf_size - w), "\n--- Log (oldest first) ---\n");
    }
    if (w < buf_size - 1) {
        VehicleSim_GetErrorLog(sim, out_buf + w, buf_size - w);
    }
}

//=========================
// 7. VALIDATION & STATS
//=========================

UNITY_API void VehicleSim_Validate(
    const VehicleSimulation* sim,
    ValidationResult* out_result
) {
    if (!out_result) return;

    out_result->is_valid   = 1;
    out_result->error_code = 0;
    memset(out_result->message, 0, sizeof(out_result->message));

    if (!sim) {
        out_result->is_valid   = 0;
        out_result->error_code = -1;
        strncpy(out_result->message, "Null simulation handle", sizeof(out_result->message) - 1);
        return;
    }

    if (!sim->vehicle) {
        out_result->is_valid   = 0;
        out_result->error_code = -3;
        strncpy(out_result->message,
            "vehicle pointer is NULL - Create() may have failed",
            sizeof(out_result->message) - 1);
        SIM_ERROR((VehicleSimulation*)sim, "Validate: vehicle pointer is NULL");
        return;
    }

    if (!sim->initialized) {
        out_result->is_valid   = 0;
        out_result->error_code = -2;
        snprintf(out_result->message, sizeof(out_result->message),
            "Simulation not initialized (step_skip_count=%d, last_error=%s)",
            sim->step_skip_count,
            sim->has_error ? sim->last_error : "(none)");
        return;
    }

    // ALPHA TODO: Check for NaN/Inf in vehicle state
    strncpy(out_result->message, "Simulation state valid", sizeof(out_result->message) - 1);
}

UNITY_API void VehicleSim_GetStats(
    const VehicleSimulation* sim,
    SimulationStats* out_stats
) {
    if (!sim || !out_stats) return;
    
    memset(out_stats, 0, sizeof(SimulationStats));
    
    out_stats->total_steps = sim->total_steps;
    out_stats->average_step_time_ms = (sim->total_steps > 0) ? 
        (sim->total_step_time / sim->total_steps) * 1000.0 : 0.0;
    out_stats->max_step_time_ms = sim->max_step_time * 1000.0;
    
    // Real-time factor: ratio of simulation time to real time
    if (sim->total_step_time > 0.0) {
        out_stats->real_time_factor = sim->current_time / sim->total_step_time;
    } else {
        out_stats->real_time_factor = 0.0;
    }
}

//=========================
// 8. DIAGNOSTICS (Internal/Debug)
//=========================

UNITY_API void VehicleSim_PrintState(const VehicleSimulation* sim) {
    if (!sim) return;
    
    VDE_LOG_INFO("=== Simulation State ===");
    VDE_LOG_INFO("Time: %.3f s", sim->current_time);
    VDE_LOG_INFO("Timestep: %.6f s", sim->timestep);
    VDE_LOG_INFO("Initialized: %d", sim->initialized);
    
    vehicle_model_print_summary(sim->vehicle);
}

UNITY_API int VehicleSim_IsInitialized(const VehicleSimulation* sim) {
    return sim ? sim->initialized : 0;
}
