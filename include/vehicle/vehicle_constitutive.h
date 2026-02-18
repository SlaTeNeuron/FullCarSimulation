#pragma once
// Vehicle Dynamics Engine - Vehicle Constitutive Equations
// Guiggiani Chapter 3, Section 3.3: "Vehicle Constitutive Equations"
//
// The CONSTITUTIVE equations describe component behavior - how forces
// depend on deformations and velocities.
//
// This is the SECOND equation set in Guiggiani's three-equation structure:
//   1. CONGRUENCE → 2. CONSTITUTIVE → 3. EQUILIBRIUM
//
// Constitutive laws in the vehicle model:
//   - Tires: Forces/moments as functions of slips (Chapter 2, 10)
//   - Springs: Forces from deflections
//   - Dampers: Forces from velocities
//   - Aerodynamics: Forces from air velocity
//   - Brakes: Torques from pressure
//
// Key Sections:
//   - 3.3: Tire constitutive equations
//   - Chapter 2: Tire mechanics and Magic Formula
//   - Chapter 10: Advanced tire models (brush model)

#include "core/math/math_base.h"
#include "core/math/vec3.h"

//-------------------------
// Forward Declarations
//-------------------------

typedef struct Vehicle Vehicle;
typedef struct TireSlips TireSlips;

//-------------------------
// Component Force/Torque Outputs
//-------------------------

// Tire forces (constitutive output)
typedef struct TireConstitutiveForces {
    vde_vec3 force;          // Force in tire frame (Fx, Fy, Fz)
    vde_vec3 moment;         // Moment about contact patch (Mx, My, Mz)
    vde_real relaxation_length;  // First-order relaxation length (m)
} TireConstitutiveForces;

// Suspension forces (constitutive output)
typedef struct SuspensionConstitutiveForces {
    vde_real spring_force;   // Spring force (N)
    vde_real damper_force;   // Damper force (N)
    vde_real total_force;    // Total suspension force (N)
    vde_real anti_roll_moment;  // Anti-roll bar moment (Nm)
} SuspensionConstitutiveForces;

// Aerodynamic forces (constitutive output)
typedef struct AeroConstitutiveForces {
    vde_vec3 force;          // Aero force (drag, side, lift)
    vde_vec3 moment;         // Aero moment (roll, pitch, yaw)
    vde_real downforce;      // Total downforce (N)
} AeroConstitutiveForces;

// Brake torques (constitutive output)
typedef struct BrakeConstitutiveTorques {
    vde_real torque[4];      // Brake torque at each wheel (Nm)
    vde_real pressure;       // Brake pressure (Pa)
} BrakeConstitutiveTorques;

//-------------------------
// Constitutive Evaluation API
//-------------------------

// Evaluate ALL constitutive equations for current vehicle state
// This computes forces/torques from current deformations and velocities
VDE_API void vehicle_constitutive_evaluate_all(Vehicle* vehicle);

//--- Individual Component Evaluation ---

// Evaluate tire constitutive equations (Chapter 2)
// Input: tire slips (from congruence equations)
// Output: tire forces and moments
VDE_API void vehicle_constitutive_evaluate_tire(
    const Vehicle* vehicle,
    int corner_index,
    const TireSlips* slips,
    TireConstitutiveForces* out_forces
);

// Evaluate suspension constitutive equations (Section 3.8)
// Input: suspension deflection and velocity
// Output: spring and damper forces
VDE_API void vehicle_constitutive_evaluate_suspension(
    const Vehicle* vehicle,
    int corner_index,
    vde_real deflection,
    vde_real velocity,
    SuspensionConstitutiveForces* out_forces
);

// Evaluate aerodynamic constitutive equations (Chapter 7)
// Input: vehicle velocity and orientation
// Output: aero forces and moments
VDE_API void vehicle_constitutive_evaluate_aero(
    const Vehicle* vehicle,
    const vde_vec3* velocity,
    AeroConstitutiveForces* out_forces
);

// Evaluate brake constitutive equations (Chapter 4)
// Input: brake pressure/input
// Output: brake torques
VDE_API void vehicle_constitutive_evaluate_brakes(
    const Vehicle* vehicle,
    vde_real brake_input,
    BrakeConstitutiveTorques* out_torques
);

//-------------------------
// Tire Constitutive Models
// (Guiggiani Chapter 2, Section 2.10 and Chapter 10)
//-------------------------

// Tire model types available
typedef enum TireModelType {
    TIRE_MODEL_MAGIC_FORMULA,    // Pacejka Magic Formula (Section 2.10)
    TIRE_MODEL_BRUSH,            // Brush model (Chapter 10)
    TIRE_MODEL_LINEAR,           // Linear model (for analysis)
    TIRE_MODEL_LOOKUP            // Lookup table from data
} TireModelType;

// Set which tire model to use for constitutive equations
VDE_API void vehicle_constitutive_set_tire_model(
    Vehicle* vehicle,
    TireModelType model_type
);

VDE_API TireModelType vehicle_constitutive_get_tire_model(
    const Vehicle* vehicle
);

//-------------------------
// Suspension Constitutive Models
// (Guiggiani Section 3.8 and Chapter 8)
//-------------------------

// Suspension model types
typedef enum SuspensionModelType {
    SUSPENSION_LINEAR,           // Linear springs and dampers
    SUSPENSION_PROGRESSIVE,      // Progressive spring rate
    SUSPENSION_INERTER           // With inerter (Section 8.2.1)
} SuspensionModelType;

// Set suspension model type
VDE_API void vehicle_constitutive_set_suspension_model(
    Vehicle* vehicle,
    SuspensionModelType model_type
);

//-------------------------
// Constitutive Parameters
//-------------------------

// Set tire parameters (for Magic Formula, brush model, etc.)
VDE_API void vehicle_constitutive_set_tire_params(
    Vehicle* vehicle,
    int corner_index,
    const void* params  // Type depends on tire model
);

// Set suspension parameters
VDE_API void vehicle_constitutive_set_suspension_params(
    Vehicle* vehicle,
    int corner_index,
    vde_real spring_stiffness,
    vde_real damping_coefficient
);

// Set aerodynamic parameters
VDE_API void vehicle_constitutive_set_aero_params(
    Vehicle* vehicle,
    vde_real drag_coeff,
    vde_real lift_coeff,
    vde_real frontal_area
);

//-------------------------
// Validation
//-------------------------

// Check that all constitutive parameters are physically valid
VDE_API int vehicle_constitutive_validate_params(const Vehicle* vehicle);

//-------------------------
// Key Concepts (from Guiggiani)
//-------------------------

// Magic Formula (Section 2.10):
//   y(x) = D * sin[C * arctan{B*x - E*(B*x - arctan(B*x))}]
//   where x is slip, y is force coefficient
//   B: stiffness factor, C: shape factor, D: peak factor, E: curvature factor

// Suspension Forces (Section 3.8):
//   Fspring = k * Δz              (spring stiffness k)
//   Fdamper = c * Δz_dot          (damping coefficient c)
//   Ftotal = Fspring + Fdamper

// Aerodynamic Forces (Chapter 7):
//   Drag = 0.5 * ρ * A * Cd * V²
//   Downforce = 0.5 * ρ * A * Cl * V²

