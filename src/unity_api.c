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

#include <stdlib.h>
#include <string.h>

//-------------------------
// Internal Simulation State
//-------------------------

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
};

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
    sim->timestep = timestep;
    sim->current_time = 0.0;
    sim->initialized = 0;
    sim->telemetry_callback = NULL;
    sim->telemetry_user_data = NULL;
    sim->telemetry_recording = 0;
    sim->telemetry_frame_count = 0;
    sim->total_steps = 0;
    sim->total_step_time = 0.0;
    sim->max_step_time = 0.0;
    
    // Create vehicle model
    sim->vehicle = vehicle_model_create();
    if (!sim->vehicle) {
        VDE_LOG_ERROR("Failed to create vehicle model");
        free(sim);
        return NULL;
    }
    
    VDE_LOG_INFO("Simulation created with timestep = %f", timestep);
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
    if (!sim || !vehicle_config_path) {
        VDE_LOG_ERROR("Invalid arguments to LoadVehicle");
        return -1;
    }
    
    // ALPHA: Use hardcoded defaults for now
    VDE_LOG_WARN("LoadVehicle: Using hardcoded defaults (parameter loading not implemented)");
    return vehicle_model_init_from_params(sim->vehicle, vehicle_config_path) ? 0 : -1;
}

UNITY_API int VehicleSim_LoadTrack(VehicleSimulation* sim, const char* track_config_path) {
    if (!sim || !track_config_path) {
        VDE_LOG_ERROR("Invalid arguments to LoadTrack");
        return -1;
    }
    
    // ALPHA: Track loading not implemented yet
    VDE_LOG_WARN("LoadTrack: Not implemented in alpha");
    return 0;
}

UNITY_API int VehicleSim_Initialize(VehicleSimulation* sim) {
    if (!sim) {
        VDE_LOG_ERROR("Invalid simulation in Initialize");
        return -1;
    }
    
    // Validate vehicle model
    if (!vehicle_model_validate(sim->vehicle)) {
        VDE_LOG_ERROR("Vehicle model validation failed");
        return -1;
    }
    
    sim->initialized = 1;
    VDE_LOG_INFO("Simulation initialized");
    return 0;
}

UNITY_API void VehicleSim_Reset(VehicleSimulation* sim) {
    if (!sim) return;
    
    sim->current_time = 0.0;
    memset(&sim->inputs, 0, sizeof(DriverInputs));
    sim->telemetry_frame_count = 0;
    sim->total_steps = 0;
    sim->total_step_time = 0.0;
    sim->max_step_time = 0.0;
    
    VDE_LOG_INFO("Simulation reset");
}

//=========================
// 2. SIMULATION CONTROL
//=========================

UNITY_API void VehicleSim_Step(VehicleSimulation* sim) {
    if (!sim || !sim->initialized) {
        return;
    }
    
    // Track step time for performance stats
    // ALPHA TODO: Use actual timing (clock_gettime or similar)
    double step_start_time = 0.0;  // Would use actual timing here
    
    // Execute one physics timestep
    vehicle_model_step(sim->vehicle, sim->timestep);
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

UNITY_API void VehicleSim_StepMultiple(VehicleSimulation* sim, int num_steps) {
    if (!sim || !sim->initialized || num_steps <= 0) {
        return;
    }
    
    for (int i = 0; i < num_steps; i++) {
        VehicleSim_Step(sim);
    }
}

UNITY_API double VehicleSim_GetTime(const VehicleSimulation* sim) {
    return sim ? sim->current_time : 0.0;
}

//=========================
// 3. INPUT INTERFACE
//=========================

UNITY_API void VehicleSim_SetInputs(VehicleSimulation* sim, const DriverInputs* inputs) {
    if (!sim || !inputs) return;
    
    sim->inputs = *inputs;
    
    // ALPHA TODO: Apply inputs to vehicle model
    // vehicle_set_inputs(sim->vehicle, inputs->throttle, inputs->brake, inputs->steering);
}

UNITY_API void VehicleSim_SetBasicInputs(
    VehicleSimulation* sim,
    double throttle,
    double brake,
    double steering
) {
    if (!sim) return;
    
    sim->inputs.throttle = throttle;
    sim->inputs.brake = brake;
    sim->inputs.steering = steering;
    sim->inputs.clutch = 0.0;  // Default: clutch engaged
    sim->inputs.gear = 1;      // Default: first gear
    
    // ALPHA TODO: Apply inputs to vehicle model
    // vehicle_set_inputs(sim->vehicle, throttle, brake, steering);
}

UNITY_API void VehicleSim_GetInputs(const VehicleSimulation* sim, DriverInputs* out_inputs) {
    if (!sim || !out_inputs) return;
    
    *out_inputs = sim->inputs;
}

//=========================
// 4. OUTPUT INTERFACE
//=========================

UNITY_API void VehicleSim_GetRenderData(const VehicleSimulation* sim, VehicleRenderData* out_data) {
    if (!sim || !out_data) return;
    
    // ALPHA: Return placeholder data
    memset(out_data, 0, sizeof(VehicleRenderData));
    
    // ALPHA TODO: Get actual vehicle state
    // VehicleState state;
    // vehicle_model_get_state(sim->vehicle, &state);
    // Copy state to render data...
}

UNITY_API void VehicleSim_GetWheelState(const VehicleSimulation* sim, int wheel_index, WheelRenderData* out_data) {
    if (!sim || !out_data || wheel_index < 0 || wheel_index >= 4) return;
    
    // ALPHA: Return placeholder data
    memset(out_data, 0, sizeof(WheelRenderData));
}

UNITY_API void VehicleSim_RegisterTelemetryCallback(
    VehicleSimulation* sim,
    TelemetryCallback callback,
    void* user_data
) {
    if (!sim) return;
    
    sim->telemetry_callback = callback;
    sim->telemetry_user_data = user_data;
    
    VDE_LOG_INFO("Telemetry callback registered");
}

UNITY_API void VehicleSim_GetTelemetry(const VehicleSimulation* sim, TelemetryFrame* out_telemetry) {
    if (!sim || !out_telemetry) return;
    
    // ALPHA: Return placeholder data
    memset(out_telemetry, 0, sizeof(TelemetryFrame));
    out_telemetry->time = sim->current_time;
    out_telemetry->delta_time = sim->timestep;
    
    // Copy current inputs
    out_telemetry->throttle = sim->inputs.throttle;
    out_telemetry->brake = sim->inputs.brake;
    out_telemetry->steering = sim->inputs.steering;
    out_telemetry->gear = sim->inputs.gear;
    
    // ALPHA TODO: Fill with actual vehicle state data
}

UNITY_API void VehicleSim_StartTelemetryRecording(VehicleSimulation* sim) {
    if (!sim) return;
    
    sim->telemetry_recording = 1;
    sim->telemetry_frame_count = 0;
    
    VDE_LOG_INFO("Telemetry recording started");
}

UNITY_API int VehicleSim_ExportTelemetryCSV(
    VehicleSimulation* sim,
    const char* output_path
) {
    if (!sim || !output_path) return -1;
    
    // ALPHA: Not implemented yet
    VDE_LOG_WARN("ExportTelemetryCSV: Not implemented in alpha");
    return -1;
}

UNITY_API int VehicleSim_GetTelemetryFrameCount(const VehicleSimulation* sim) {
    return sim ? sim->telemetry_frame_count : 0;
}

UNITY_API void VehicleSim_ClearTelemetry(VehicleSimulation* sim) {
    if (!sim) return;
    
    sim->telemetry_frame_count = 0;
    sim->telemetry_recording = 0;
    
    VDE_LOG_INFO("Telemetry cleared");
}

//=========================
// 5. ASSET MANAGEMENT
//=========================

UNITY_API void VehicleSim_GetVehicleParameters(const VehicleSimulation* sim, VehicleParameters* out_params) {
    if (!sim || !out_params) return;
    
    // ALPHA: Return placeholder parameters
    memset(out_params, 0, sizeof(VehicleParameters));
    out_params->mass = 600.0;
    out_params->wheelbase = 1.55;
    out_params->track_front = 1.2;
    out_params->track_rear = 1.2;
    out_params->cg_height = 0.3;
    out_params->tire_radius = 0.2;
    out_params->tire_friction_coeff = 1.0;
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
    
    // ALPHA: Return flat ground with standard friction
    query->normal[0] = 0.0;
    query->normal[1] = 1.0;  // Up vector
    query->normal[2] = 0.0;
    query->friction_coefficient = 1.0;
    query->elevation = 0.0;
    query->is_valid = 1;
}

//=========================
// 6. ERROR HANDLING
//=========================

UNITY_API const char* VehicleSim_GetLastError(const VehicleSimulation* sim) {
    if (!sim) return "Invalid simulation handle";
    // ALPHA: Return generic message
    return "No error information available (alpha)";
}

UNITY_API void VehicleSim_ClearError(VehicleSimulation* sim) {
    if (!sim) return;
    // ALPHA: No-op
}

//=========================
// 7. VALIDATION & STATS
//=========================

UNITY_API void VehicleSim_Validate(
    const VehicleSimulation* sim,
    ValidationResult* out_result
) {
    if (!out_result) return;
    
    // Initialize result
    out_result->is_valid = 1;
    out_result->error_code = 0;
    memset(out_result->message, 0, sizeof(out_result->message));
    
    if (!sim) {
        out_result->is_valid = 0;
        out_result->error_code = -1;
        strncpy(out_result->message, "Null simulation handle", sizeof(out_result->message) - 1);
        return;
    }
    
    if (!sim->initialized) {
        out_result->is_valid = 0;
        out_result->error_code = -2;
        strncpy(out_result->message, "Simulation not initialized", sizeof(out_result->message) - 1);
        return;
    }
    
    // ALPHA TODO: Add more validation checks
    // - Check for NaN/Inf in vehicle state
    // - Check for unreasonable values
    // - Check vehicle model integrity
    
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
