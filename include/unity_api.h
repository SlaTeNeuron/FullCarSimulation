#pragma once
// Vehicle Dynamics Engine - Unity DLL Interface
// 
// This header defines the COMPLETE external API for the Unity simulation.
// All functions here are exported from the DLL.
//
// ARCHITECTURE:
//   Unity (C#) → DLL Boundary (this file) → Internal Physics (vehicle_model.h, etc.)
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
// 2. SIMULATION CONTROL
//=========================

/// Step simulation forward by one physics timestep
/// Call this in a loop at the physics rate (e.g., 1000 Hz)
UNITY_API void VehicleSim_Step(VehicleSimulation* sim);

/// Step simulation multiple times (for catching up)
/// @param num_steps Number of physics steps to execute
UNITY_API void VehicleSim_StepMultiple(VehicleSimulation* sim, int num_steps);

/// Get current simulation time
UNITY_API double VehicleSim_GetTime(const VehicleSimulation* sim);

//=========================
// 3. INPUT INTERFACE (Unity → DLL)
//=========================

/// Driver control inputs structure
typedef struct DriverInputs {
    double throttle;        // [0, 1] - 0 = no throttle, 1 = full throttle
    double brake;           // [0, 1] - 0 = no brake, 1 = full brake
    double steering;        // [-1, 1] - negative = left, positive = right
    double clutch;          // [0, 1] - 0 = engaged, 1 = disengaged
    int gear;               // -1 = reverse, 0 = neutral, 1-6 = forward gears
} DriverInputs;

/// Set driver inputs
UNITY_API void VehicleSim_SetInputs(VehicleSimulation* sim, const DriverInputs* inputs);

/// Convenience: Set basic inputs (throttle, brake, steering only)
UNITY_API void VehicleSim_SetBasicInputs(
    VehicleSimulation* sim,
    double throttle,
    double brake,
    double steering
);

//=========================
// 4. RENDERING DATA (DLL → Unity)
//=========================

/// Vehicle chassis state for Unity rendering
typedef struct ChassisRenderData {
    // Position (world space, meters)
    double position_x;
    double position_y;
    double position_z;
    
    // Orientation (quaternion: w, x, y, z)
    double orientation_w;
    double orientation_x;
    double orientation_y;
    double orientation_z;
    
    // Velocities (for motion blur, particle effects)
    double velocity_x;      // m/s
    double velocity_y;
    double velocity_z;
    
    double angular_vel_x;   // rad/s
    double angular_vel_y;
    double angular_vel_z;
} ChassisRenderData;

/// Individual wheel state for Unity rendering
typedef struct WheelRenderData {
    // Position (world space, meters)
    double position_x;
    double position_y;
    double position_z;
    
    // Rotation angle (radians, for spinning animation)
    double rotation_angle;
    
    // Steering angle (radians, for visual steering)
    double steer_angle;
    
    // Suspension deflection (meters, for suspension animation)
    double suspension_deflection;
    
    // Tire slip (for particle effects, tire smoke)
    double longitudinal_slip;  // dimensionless
    double lateral_slip;       // radians
    
    // Tire forces (for debugging visualization)
    double tire_force_x;       // N
    double tire_force_y;       // N
    double tire_force_z;       // N (normal load)
    
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

/// Comprehensive telemetry data for analysis and dataset generation
typedef struct TelemetryFrame {
    // Time
    double time;                       // seconds
    double delta_time;                 // seconds since last frame
    
    // Chassis state
    double position[3];                // x, y, z (m)
    double velocity[3];                // vx, vy, vz (m/s)
    double acceleration[3];            // ax, ay, az (m/s²)
    double orientation[4];             // quaternion: w, x, y, z
    double angular_velocity[3];        // p, q, r (rad/s)
    double angular_acceleration[3];    // p_dot, q_dot, r_dot (rad/s²)
    
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
    double start_position[3];          // x, y, z starting position
    double start_orientation[4];       // quaternion: w, x, y, z
} TrackInfo;

/// Get track information (after loading)
UNITY_API void VehicleSim_GetTrackInfo(
    const VehicleSimulation* sim,
    TrackInfo* out_info
);

/// Query track surface properties at a position
typedef struct SurfaceQuery {
    double position[3];                // Input: query position (x, y, z)
    double normal[3];                  // Output: surface normal
    double friction_coefficient;       // Output: friction at this point
    double elevation;                  // Output: elevation (z coordinate)
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

/// Get last error message
/// @return Null-terminated error string, or NULL if no error
UNITY_API const char* VehicleSim_GetLastError(const VehicleSimulation* sim);

/// Clear error state
UNITY_API void VehicleSim_ClearError(VehicleSimulation* sim);

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
using System.Runtime.InteropServices;

public class VehicleSimulationWrapper 
{
    [DllImport("racing_sim_plugin")]
    private static extern IntPtr VehicleSim_Create(double timestep);
    
    [DllImport("racing_sim_plugin")]
    private static extern void VehicleSim_Destroy(IntPtr sim);
    
    [DllImport("racing_sim_plugin")]
    private static extern int VehicleSim_LoadVehicle(IntPtr sim, string vehiclePath);
    
    [DllImport("racing_sim_plugin")]
    private static extern int VehicleSim_LoadTrack(IntPtr sim, string trackPath);
    
    [DllImport("racing_sim_plugin")]
    private static extern int VehicleSim_Initialize(IntPtr sim);
    
    [DllImport("racing_sim_plugin")]
    private static extern void VehicleSim_Step(IntPtr sim);
    
    [DllImport("racing_sim_plugin")]
    private static extern void VehicleSim_SetBasicInputs(IntPtr sim, double throttle, double brake, double steer);
    
    [DllImport("racing_sim_plugin")]
    private static extern void VehicleSim_GetRenderData(IntPtr sim, out VehicleRenderData data);
    
    private IntPtr simHandle;
    
    void Start() {
        // Create simulation (1 ms timestep)
        simHandle = VehicleSim_Create(0.001);
        
        // Load assets
        VehicleSim_LoadVehicle(simHandle, "data/vehicles/TBReCar.txt");
        VehicleSim_LoadTrack(simHandle, "data/tracks/skidpad.txt");
        
        // Initialize
        VehicleSim_Initialize(simHandle);
    }
    
    void FixedUpdate() {
        // Get input from player
        float throttle = Input.GetAxis("Throttle");
        float brake = Input.GetAxis("Brake");
        float steering = Input.GetAxis("Steering");
        
        // Send to simulation
        VehicleSim_SetBasicInputs(simHandle, throttle, brake, steering);
        
        // Step physics
        VehicleSim_Step(simHandle);
    }
    
    void Update() {
        // Get render data
        VehicleRenderData renderData;
        VehicleSim_GetRenderData(simHandle, out renderData);
        
        // Update Unity transforms
        transform.position = new Vector3(
            (float)renderData.chassis.position_x,
            (float)renderData.chassis.position_y,
            (float)renderData.chassis.position_z
        );
        
        transform.rotation = new Quaternion(
            (float)renderData.chassis.orientation_x,
            (float)renderData.chassis.orientation_y,
            (float)renderData.chassis.orientation_z,
            (float)renderData.chassis.orientation_w
        );
        
        // Update wheels...
    }
    
    void OnDestroy() {
        VehicleSim_Destroy(simHandle);
    }
}
*/
