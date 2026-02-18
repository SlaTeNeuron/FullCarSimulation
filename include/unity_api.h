#pragma once
// Vehicle Dynamics Engine - Unity DLL Interface
// 
// This header defines the COMPLETE external API for the Unity simulation.
// All functions here are exported from the DLL.
//
// ARCHITECTURE:
//   Unity (C#) → DLL Boundary (this file) → Internal Physics (vehicle_model.h, etc.)
//
// COORDINATE SYSTEM:
//   All positions, velocities, orientations, and normals exposed through this API
//   are expressed in Unity world space:
//     X = right   (lateral)
//     Y = up      (vertical)    ← left-handed system
//     Z = forward (longitudinal)
//   The DLL converts from the internal physics convention (X=forward, Y=down, Z=right,
//   right-handed) at the API boundary — Unity consumers do not need to remap axes.
//
// DESIGN PRINCIPLES:
//   - Simple C types for easy marshaling to C#
//   - Grouped by responsibility (Lifecycle, Inputs, Outputs, Assets)
//   - All rendering data in Unity-friendly format (positions, orientations)
//   - Comprehensive telemetry for dataset generation
//   - Clear asset management (maps, vehicle configs)

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//-------------------------
// DLL Export Configuration
//-------------------------

#ifdef _WIN32
    #ifdef BUILDING_DLL
        #define UNITY_API __declspec(dllexport)
    #else
        #define UNITY_API __declspec(dllimport)
    #endif
#else
    #define UNITY_API
#endif

//-------------------------
// Opaque Handle
//-------------------------

typedef struct VehicleSimulation VehicleSimulation;

//=========================
// 1. LIFECYCLE MANAGEMENT
//=========================

/// Create simulation instance
/// @param timestep Fixed physics timestep in seconds (e.g., 0.001 for 1ms)
/// @return Opaque simulation handle, NULL on failure
UNITY_API VehicleSimulation* VehicleSim_Create(double timestep);

/// Destroy simulation instance
UNITY_API void VehicleSim_Destroy(VehicleSimulation* sim);

/// Load vehicle parameters from file
/// @param vehicle_config_path Path to vehicle parameter file (e.g., "data/vehicles/TBReCar.txt")
/// @return 0 on success, error code on failure
UNITY_API int VehicleSim_LoadVehicle(VehicleSimulation* sim, const char* vehicle_config_path);

/// Load track/map data
/// @param track_config_path Path to track definition file (e.g., "data/tracks/skidpad.txt")
/// @return 0 on success, error code on failure
UNITY_API int VehicleSim_LoadTrack(VehicleSimulation* sim, const char* track_config_path);

/// Initialize simulation (call after loading vehicle and track)
/// @return 0 on success, error code on failure
UNITY_API int VehicleSim_Initialize(VehicleSimulation* sim);

/// Reset simulation to initial state
UNITY_API void VehicleSim_Reset(VehicleSimulation* sim);

//=========================
// 2. INPUT STRUCTURES
//=========================

/// Driver control inputs structure
typedef struct DriverInputs {
    double throttle;        // [0, 1] - 0 = no throttle, 1 = full throttle
    double brake;           // [0, 1] - 0 = no brake, 1 = full brake
    double steering;        // [-1, 1] - negative = left, positive = right
    double clutch;          // [0, 1] - 0 = engaged, 1 = disengaged
    int gear;               // -1 = reverse, 0 = neutral, 1-6 = forward gears
} DriverInputs;

//=========================
// 3. SIMULATION CONTROL
//=========================

/// Step simulation forward by one physics timestep with the given inputs
/// This is the primary stepping function - inputs are applied directly for this step
/// @param inputs Driver control inputs for this timestep
/// Call this in a loop at the physics rate (e.g., 1000 Hz)
UNITY_API void VehicleSim_Step(VehicleSimulation* sim, const DriverInputs* inputs);

/// Step simulation multiple times with the same inputs (for catching up)
/// @param inputs Driver control inputs to use for all steps
/// @param num_steps Number of physics steps to execute
UNITY_API void VehicleSim_StepMultiple(VehicleSimulation* sim, const DriverInputs* inputs, int num_steps);

/// Get current simulation time
UNITY_API double VehicleSim_GetTime(const VehicleSimulation* sim);

//=========================
// 3a. INPUT UTILITIES
//=========================

/// Get last applied driver inputs (for debugging)
UNITY_API void VehicleSim_GetInputs(const VehicleSimulation* sim, DriverInputs* out_inputs);

//=========================
// 4. RENDERING DATA (DLL → Unity)
//=========================

/// Vehicle chassis state for Unity rendering.
/// All fields are in Unity world space: X=right, Y=up, Z=forward (left-handed).
typedef struct ChassisRenderData {
    // Position (Unity world space, meters)
    double position_x;      // +X right
    double position_y;      // +Y up
    double position_z;      // +Z forward
    
    // Orientation quaternion (Unity convention: w, x, y, z)
    double orientation_w;
    double orientation_x;
    double orientation_y;
    double orientation_z;
    
    // Linear velocity (Unity world space, m/s)
    double velocity_x;
    double velocity_y;
    double velocity_z;
    
    // Angular velocity (Unity world space, rad/s, left-hand rule)
    double angular_vel_x;
    double angular_vel_y;
    double angular_vel_z;
} ChassisRenderData;

/// Individual wheel state for Unity rendering.
/// All positional fields are in Unity world space: X=right, Y=up, Z=forward.
typedef struct WheelRenderData {
    // Position (Unity world space, meters)
    double position_x;      // +X right
    double position_y;      // +Y up
    double position_z;      // +Z forward
    
    // Rotation angle (radians, for spinning animation)
    double rotation_angle;
    
    // Steering angle (radians, for visual steering)
    double steer_angle;
    
    // Suspension deflection (meters, for suspension animation)
    double suspension_deflection;
    
    // Tire slip (for particle effects, tire smoke)
    double longitudinal_slip;  // dimensionless
    double lateral_slip;       // radians
    
    // Tire forces (Unity world space, N)
    double tire_force_x;       // N  (+X right)
    double tire_force_y;       // N  (+Y up / normal load)
    double tire_force_z;       // N  (+Z forward)
    
    // Contact state
    int is_grounded;           // 0 = airborne, 1 = contact
} WheelRenderData;

/// Complete vehicle render state (all data needed for Unity visualization)
typedef struct VehicleRenderData {
    double simulation_time;            // seconds
    ChassisRenderData chassis;
    WheelRenderData wheels[4];         // FL, FR, RL, RR
    
    // Visual feedback data
    double speed_kmh;                  // km/h (for speedometer)
    double rpm;                        // engine RPM (for tachometer)
    int current_gear;                  // current gear
    double steering_wheel_angle;       // radians (for steering wheel visual)
    
    // Aerodynamic effects (for particle systems)
    double aero_drag_force;            // N
    double aero_downforce;             // N
} VehicleRenderData;

/// Get complete vehicle state for rendering
/// This is the PRIMARY function Unity calls every frame for visualization
UNITY_API void VehicleSim_GetRenderData(
    const VehicleSimulation* sim,
    VehicleRenderData* out_data
);

//=========================
// 5. TELEMETRY OUTPUT (DLL → Unity/File)
//=========================

/// Comprehensive telemetry data for analysis and dataset generation.
/// All vector quantities are in Unity world space: X=right, Y=up, Z=forward.
typedef struct TelemetryFrame {
    // Time
    double time;                       // seconds
    double delta_time;                 // seconds since last frame
    
    // Chassis state (Unity world space)
    double position[3];                // [x_right, y_up, z_forward] (m)
    double velocity[3];                // [vx, vy, vz] (m/s)
    double acceleration[3];            // [ax, ay, az] (m/s²)
    double orientation[4];             // quaternion [w, x, y, z] (Unity convention)
    double angular_velocity[3];        // [wx, wy, wz] (rad/s, left-hand rule)
    double angular_acceleration[3];    // [alpha_x, alpha_y, alpha_z] (rad/s²)
    
    // Derived quantities
    double speed;                      // magnitude of velocity (m/s)
    double yaw_rate;                   // rad/s
    double lateral_acceleration;       // m/s²
    double longitudinal_acceleration;  // m/s²
    
    // Driver inputs
    double throttle;                   // [0, 1]
    double brake;                      // [0, 1]
    double steering;                   // [-1, 1]
    int gear;
    
    // Tire data (4 wheels: FL, FR, RL, RR)
    double tire_slip_ratio[4];         // longitudinal slip
    double tire_slip_angle[4];         // radians
    double tire_vertical_load[4];      // N
    double tire_force_long[4];         // N
    double tire_force_lat[4];          // N
    double tire_force_vert[4];         // N
    
    // Suspension data
    double suspension_deflection[4];   // m
    double suspension_velocity[4];     // m/s
    double suspension_force[4];        // N
    
    // Load transfers (Guiggiani Section 3.7)
    double longitudinal_load_transfer; // N
    double lateral_load_transfer_front;// N
    double lateral_load_transfer_rear; // N
    
    // Driveline
    double engine_rpm;
    double engine_torque;              // Nm
    double wheel_angular_vel[4];       // rad/s
    
    // Aerodynamics
    double aero_drag;                  // N
    double aero_downforce;             // N
    double aero_side_force;            // N
    
    // Energy (for analysis)
    double kinetic_energy;             // J
    double power_output;               // W
} TelemetryFrame;

/// Telemetry callback function type
/// Unity can register a callback to receive telemetry data each step
typedef void (*TelemetryCallback)(const TelemetryFrame* frame, void* user_data);

/// Register telemetry callback
/// @param callback Function to call each simulation step with telemetry data
/// @param user_data Opaque pointer passed back to callback (can be NULL)
UNITY_API void VehicleSim_RegisterTelemetryCallback(
    VehicleSimulation* sim,
    TelemetryCallback callback,
    void* user_data
);

/// Get latest telemetry frame (without callback)
UNITY_API void VehicleSim_GetTelemetry(
    const VehicleSimulation* sim,
    TelemetryFrame* out_frame
);

/// Start recording telemetry to memory buffer
UNITY_API void VehicleSim_StartTelemetryRecording(VehicleSimulation* sim);

/// Stop recording and export to CSV file
/// @param output_path Path for output file (e.g., "telemetry/run_001.csv")
/// @return 0 on success, error code on failure
UNITY_API int VehicleSim_ExportTelemetryCSV(
    VehicleSimulation* sim,
    const char* output_path
);

/// Get number of recorded telemetry frames
UNITY_API int VehicleSim_GetTelemetryFrameCount(const VehicleSimulation* sim);

/// Clear recorded telemetry
UNITY_API void VehicleSim_ClearTelemetry(VehicleSimulation* sim);

//=========================
// 6. ASSET MANAGEMENT
//=========================

/// Vehicle parameter structure (read-only access to loaded vehicle data)
typedef struct VehicleParameters {
    // Mass properties
    double mass;                       // kg
    double wheelbase;                  // m
    double track_front;                // m
    double track_rear;                 // m
    double cg_height;                  // m
    double cg_to_front;                // m
    double cg_to_rear;                 // m
    
    // Inertia
    double inertia_roll;               // kg·m²
    double inertia_pitch;              // kg·m²
    double inertia_yaw;                // kg·m²
    
    // Tire properties (simplified)
    double tire_radius;                // m
    double tire_friction_coeff;        // dimensionless
    
    // Suspension
    double spring_rate_front;          // N/m
    double spring_rate_rear;           // N/m
    double damping_front;              // N·s/m
    double damping_rear;               // N·s/m
    
    // Aerodynamics
    double drag_coefficient;
    double lift_coefficient;
    double frontal_area;               // m²
    
    // Engine
    double max_engine_torque;          // Nm
    double max_rpm;
} VehicleParameters;

/// Get vehicle parameters (after loading)
UNITY_API void VehicleSim_GetVehicleParameters(
    const VehicleSimulation* sim,
    VehicleParameters* out_params
);

/// Track/map information
typedef struct TrackInfo {
    char name[256];                    // Track name
    double length;                     // m
    int num_sectors;                   // Number of track sectors
    double start_position[3];          // Unity world space: [x_right, y_up, z_forward] (m)
    double start_orientation[4];       // Unity quaternion: [w, x, y, z]
} TrackInfo;

/// Get track information (after loading)
UNITY_API void VehicleSim_GetTrackInfo(
    const VehicleSimulation* sim,
    TrackInfo* out_info
);

/// Query track surface properties at a position.
/// All position and normal values are in Unity world space (X=right, Y=up, Z=forward).
typedef struct SurfaceQuery {
    double position[3];                // Input:  query position [x_right, y_up, z_forward] (m)
    double normal[3];                  // Output: surface normal (Unity world space)
    double friction_coefficient;       // Output: friction at this point
    double elevation;                  // Output: Y elevation (m)
    int is_valid;                      // Output: 1 if query succeeded
} SurfaceQuery;

/// Query track surface at a position (for Unity terrain interaction)
UNITY_API void VehicleSim_QuerySurface(
    const VehicleSimulation* sim,
    SurfaceQuery* query
);

//=========================
// 7. ERROR HANDLING
//=========================

/// Error severity levels
typedef enum SimErrorLevel {
    SIM_LOG_DEBUG   = 0,   // Verbose diagnostics
    SIM_LOG_INFO    = 1,   // Normal operational messages
    SIM_LOG_WARN    = 2,   // Non-fatal problems (e.g. alpha stubs)
    SIM_LOG_ERROR   = 3    // Failures that prevent correct operation
} SimErrorLevel;

/// Real-time log callback - Unity subscribes to receive every logged message
/// as it is emitted, including inside VehicleSim_Step.
/// @param level   Severity of the message
/// @param message Null-terminated, UTF-8 string
/// @param user_data Opaque pointer supplied at registration
typedef void (*SimLogCallback)(SimErrorLevel level, const char* message, void* user_data);

/// Register a callback that receives every log message in real time.
/// Pass NULL to unregister.  Safe to call before Initialize.
/// Unity C#: use [MonoPInvokeCallback] and DontDestroyOnLoad on the delegate.
UNITY_API void VehicleSim_SetLogCallback(
    VehicleSimulation* sim,
    SimLogCallback callback,
    void* user_data
);

/// Get last error message.  Returns a non-empty string whenever an error was
/// recorded since the last VehicleSim_ClearError call.
/// The string is owned by the sim; copy it before the next API call.
/// @return Null-terminated error string (never NULL)
UNITY_API const char* VehicleSim_GetLastError(const VehicleSimulation* sim);

/// Clear error state (last error string and the full log)
UNITY_API void VehicleSim_ClearError(VehicleSimulation* sim);

/// Copy the full recent error log into caller-supplied buffer.
/// Each entry is one line: "[T=nnn.nnn][LEVEL] message\n"
/// Up to the last 32 errors are retained (oldest dropped first).
/// @param out_buf   Caller-allocated buffer to receive the log text
/// @param buf_size  Size of out_buf in bytes
UNITY_API void VehicleSim_GetErrorLog(
    const VehicleSimulation* sim,
    char* out_buf,
    int buf_size
);

/// Copy a complete human-readable diagnostic snapshot into caller-supplied buffer.
/// Includes: init state, timestep, total steps, sim time, vehicle pointer,
/// last error, and the full error log.  Useful as the first thing to read
/// when debugging a silent failure.
/// @param out_buf   Caller-allocated buffer (recommend >= 4096 bytes)
/// @param buf_size  Size of out_buf in bytes
UNITY_API void VehicleSim_GetDiagnostics(
    const VehicleSimulation* sim,
    char* out_buf,
    int buf_size
);

//=========================
// 8. DEBUGGING & VALIDATION
//=========================

/// Validation result structure
typedef struct ValidationResult {
    int is_valid;                      // 0 = invalid, 1 = valid
    char message[512];                 // Validation message
    int error_code;                    // Specific error code
} ValidationResult;

/// Validate current simulation state (check for NaN, unreasonable values, etc.)
UNITY_API void VehicleSim_Validate(
    const VehicleSimulation* sim,
    ValidationResult* out_result
);

/// Get statistics for performance monitoring
typedef struct SimulationStats {
    double average_step_time_ms;       // Average physics step time
    double max_step_time_ms;           // Maximum step time seen
    int total_steps;                   // Total steps executed
    double real_time_factor;           // Ratio of sim time to real time
} SimulationStats;

UNITY_API void VehicleSim_GetStats(
    const VehicleSimulation* sim,
    SimulationStats* out_stats
);

#ifdef __cplusplus
}
#endif

//=========================
// USAGE EXAMPLE (Unity C#)
//=========================

/*
using System;
using System.Runtime.InteropServices;
using AOT; // MonoPInvokeCallback

public class VehicleSimulationWrapper : MonoBehaviour
{
    // DLL output name is "racing_sim" (set by CMakeLists.txt OUTPUT_NAME)
    [DllImport("racing_sim")] private static extern IntPtr VehicleSim_Create(double timestep);
    [DllImport("racing_sim")] private static extern void   VehicleSim_Destroy(IntPtr sim);
    [DllImport("racing_sim")] private static extern int    VehicleSim_LoadVehicle(IntPtr sim, string path);
    [DllImport("racing_sim")] private static extern int    VehicleSim_LoadTrack(IntPtr sim, string path);
    [DllImport("racing_sim")] private static extern int    VehicleSim_Initialize(IntPtr sim);
    [DllImport("racing_sim")] private static extern void   VehicleSim_Step(IntPtr sim, ref DriverInputs inputs);
    [DllImport("racing_sim")] private static extern void   VehicleSim_GetRenderData(IntPtr sim, out VehicleRenderData data);
    [DllImport("racing_sim")] private static extern IntPtr VehicleSim_GetLastError(IntPtr sim);
    [DllImport("racing_sim")] private static extern void   VehicleSim_ClearError(IntPtr sim);
    [DllImport("racing_sim")] private static extern void   VehicleSim_GetErrorLog(IntPtr sim, System.Text.StringBuilder buf, int size);
    [DllImport("racing_sim")] private static extern void   VehicleSim_GetDiagnostics(IntPtr sim, System.Text.StringBuilder buf, int size);

    // --- Log callback ---
    // int matches SimErrorLevel: 0=DEBUG, 1=INFO, 2=WARN, 3=ERROR
    delegate void SimLogCallback(int level, IntPtr message, IntPtr userData);
    [DllImport("racing_sim")] private static extern void VehicleSim_SetLogCallback(IntPtr sim, SimLogCallback cb, IntPtr userData);

    [MonoPInvokeCallback(typeof(SimLogCallback))]
    private static void OnSimLog(int level, IntPtr messagePtr, IntPtr _) {
        string msg = Marshal.PtrToStringAnsi(messagePtr) ?? "<null>";
        string prefix = level >= 3 ? "[SIM ERROR]" : level == 2 ? "[SIM WARN]" : "[SIM]";
        if (level >= 3) Debug.LogError(prefix + " " + msg);
        else if (level == 2) Debug.LogWarning(prefix + " " + msg);
        else Debug.Log(prefix + " " + msg);
    }
    // Keep a static reference so the GC won't collect the delegate
    private static SimLogCallback _logCallbackRef = OnSimLog;

    private IntPtr simHandle;

    void Start() {
        simHandle = VehicleSim_Create(0.001); // 1 ms timestep
        if (simHandle == IntPtr.Zero) { Debug.LogError("VehicleSim_Create returned NULL"); return; }

        // Wire up real-time log callback BEFORE any other call
        VehicleSim_SetLogCallback(simHandle, _logCallbackRef, IntPtr.Zero);

        int err;
        err = VehicleSim_LoadVehicle(simHandle, "data/vehicles/TBReCar.txt");
        if (err != 0) { LogAndDump("LoadVehicle failed"); return; }

        err = VehicleSim_LoadTrack(simHandle, "data/tracks/skidpad.txt");
        if (err != 0) { LogAndDump("LoadTrack failed"); return; }

        err = VehicleSim_Initialize(simHandle);
        if (err != 0) { LogAndDump("Initialize failed"); return; }
    }

    void FixedUpdate() {
        if (simHandle == IntPtr.Zero) return;
        DriverInputs inputs = new DriverInputs {
            throttle = Input.GetAxis("Throttle"),
            brake    = Input.GetAxis("Brake"),
            steering = Input.GetAxis("Horizontal"),
            clutch   = 0.0, gear = 1
        };
        VehicleSim_Step(simHandle, ref inputs);
    }

    void Update() {
        if (simHandle == IntPtr.Zero) return;
        VehicleRenderData data;
        VehicleSim_GetRenderData(simHandle, out data);
        // Positions and quaternions are already in Unity world space (X=right, Y=up, Z=forward).
        // Assign directly — no axis remapping required.
        transform.position = new Vector3((float)data.chassis.position_x,
                                         (float)data.chassis.position_y,
                                         (float)data.chassis.position_z);
        transform.rotation = new Quaternion((float)data.chassis.orientation_x,
                                            (float)data.chassis.orientation_y,
                                            (float)data.chassis.orientation_z,
                                            (float)data.chassis.orientation_w);
    }

    void OnDestroy() { if (simHandle != IntPtr.Zero) VehicleSim_Destroy(simHandle); }

    // Helper: print the full diagnostic dump to the Unity console
    void LogAndDump(string context) {
        string lastErr = Marshal.PtrToStringAnsi(VehicleSim_GetLastError(simHandle)) ?? "<null>";
        Debug.LogError($"[VehicleSim] {context} | LastError: {lastErr}");

        var diag = new System.Text.StringBuilder(4096);
        VehicleSim_GetDiagnostics(simHandle, diag, diag.Capacity);
        Debug.LogError("[VehicleSim] Diagnostics:\n" + diag.ToString());
    }
}
*/
