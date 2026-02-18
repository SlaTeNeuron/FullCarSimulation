#pragma once
// Vehicle Dynamics Engine - Complete Vehicle Model
// Guiggiani Chapter 3: "Vehicle Model for Handling and Performance"
//
// This header defines the complete vehicle model structure following
// Guiggiani's three-equation methodology (Section 3.12):
//
//   1. CONGRUENCE (Kinematic) Equations - How vehicle parts move
//   2. CONSTITUTIVE Equations - Component behavior (tires, springs)
//   3. EQUILIBRIUM Equations - Force and moment balance
//
// References:
//   - Section 3.11: Complete vehicle model structure
//   - Section 3.12: The three-equation framework
//   - Chapters 6-7: Application to handling analysis
//   - Chapter 9: Extension to full 3D dynamics with roll

#include "core/math/math_base.h"
#include "core/math/vec3.h"
#include "core/math/quat.h"
#include "core/math/frames.h"

//-------------------------
// Forward Declarations
//-------------------------

typedef struct Vehicle Vehicle;
typedef struct VehicleState VehicleState;

//-------------------------
// Vehicle State Structure
// (Generalized coordinates q and velocities q_dot)
//-------------------------

// Complete vehicle state following Guiggiani's generalized coordinates
typedef struct VehicleState {
    //--- Position and Orientation (Generalized Coordinates q) ---
    vde_vec3 position;           // Center of mass position (world frame)
    vde_quat orientation;        // Body orientation (world frame)
    
    //--- Linear and Angular Velocities (q_dot) ---
    vde_vec3 velocity;           // Linear velocity (world frame)
    vde_vec3 angular_velocity;   // Angular velocity (body frame)
    
    //--- Wheel States ---
    vde_real wheel_angular_vel[4];  // Wheel angular velocities (rad/s)
    
    //--- Suspension Deflections ---
    vde_real suspension_deflection[4];  // Suspension travel (m)
    
    //--- Time ---
    vde_real time;
} VehicleState;

//-------------------------
// Three-Equation Structure
// (Guiggiani Section 3.12)
//-------------------------

// 1. CONGRUENCE (Kinematic) Equations - Section 3.2
//    Relates velocities, accelerations, and tire slips to vehicle motion
//    See: vehicle_congruence.h, wheel.h, suspension.h

// 2. CONSTITUTIVE Equations - Section 3.3
//    Component behavior laws (tire forces, spring forces, damper forces)
//    See: tire_models/, suspension.h, brakes.h, aerodynamics.h

// 3. EQUILIBRIUM Equations - Sections 3.4-3.6, 3.11
//    Force and moment balance on vehicle
//    See: core/physics/equations_of_motion.h, sprung_mass.h

//-------------------------
// Vehicle Model API
//-------------------------

// Create complete vehicle model with all subsystems
VDE_API Vehicle* vehicle_model_create(void);
VDE_API void vehicle_model_destroy(Vehicle* vehicle);

// Initialize from parameters (Guiggiani-style parameter sets)
VDE_API int vehicle_model_init_from_params(Vehicle* vehicle, const char* param_file);

//--- State Access ---

VDE_API void vehicle_model_get_state(const Vehicle* vehicle, VehicleState* out_state);
VDE_API void vehicle_model_set_state(Vehicle* vehicle, const VehicleState* state);

//--- Three-Equation Model Execution ---

// Step 1: Compute CONGRUENCE (kinematics)
// Calculates tire slips, suspension velocities, etc. from vehicle motion
VDE_API void vehicle_model_compute_congruence(Vehicle* vehicle);

// Step 2: Evaluate CONSTITUTIVE equations
// Computes tire forces, spring forces, damper forces from current state
VDE_API void vehicle_model_compute_constitutive(Vehicle* vehicle);

// Step 3: Assemble EQUILIBRIUM equations
// Builds equations of motion: M * q_ddot = Q(q, q_dot)
VDE_API void vehicle_model_assemble_equilibrium(Vehicle* vehicle, vde_real* out_accelerations);

// Combined step: Execute complete model for one timestep
VDE_API void vehicle_model_step(Vehicle* vehicle, vde_real dt);

//-------------------------
// Load Transfer Analysis
// (Guiggiani Section 3.7)
//-------------------------

// Compute load transfers for current vehicle state
typedef struct LoadTransfers {
    vde_real longitudinal;       // Longitudinal load transfer (N)
    vde_real lateral_front;      // Front lateral load transfer (N)
    vde_real lateral_rear;       // Rear lateral load transfer (N)
    vde_real vertical_loads[4];  // Vertical load on each tire (N)
} LoadTransfers;

VDE_API void vehicle_model_compute_load_transfers(
    const Vehicle* vehicle,
    LoadTransfers* out_transfers
);

//-------------------------
// Handling Analysis Tools
// (Guiggiani Chapters 6-7)
//-------------------------

// Compute vehicle handling characteristics
typedef struct HandlingCharacteristics {
    vde_real understeer_gradient;    // Understeer gradient (rad/g)
    vde_real critical_speed;         // Critical speed (m/s)
    vde_real characteristic_speed;   // Characteristic speed (m/s)
    vde_real yaw_gain;              // Yaw velocity gain (rad/m)
    vde_real sideslip_gain;         // Sideslip angle gain (rad/m)
} HandlingCharacteristics;

VDE_API void vehicle_model_compute_handling(
    const Vehicle* vehicle,
    HandlingCharacteristics* out_handling
);

//-------------------------
// Validation and Diagnostics
//-------------------------

// Check model internal consistency
VDE_API int vehicle_model_validate(const Vehicle* vehicle);

// Get human-readable model summary
VDE_API void vehicle_model_print_summary(const Vehicle* vehicle);

//-------------------------
// References to Guiggiani
//-------------------------

// This model implements:
//   - Chapter 2: Tire mechanics (via tire_models/)
//   - Chapter 3: Complete vehicle model (this file)
//   - Chapter 4: Braking (via brakes.h)
//   - Chapter 6: Road car handling (open differential)
//   - Chapter 7: Race car handling (locked differential)
//   - Chapter 8: Ride comfort (via suspension.h)
//   - Chapter 9: Full 3D dynamics (via sprung_mass.h, core/math/frames.h)

