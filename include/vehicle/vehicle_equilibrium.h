#pragma once
// Vehicle Dynamics Engine - Vehicle Equilibrium Equations
// Guiggiani Chapter 3, Sections 3.4-3.6, 3.11: "Vehicle Equilibrium Equations"
//
// The EQUILIBRIUM equations express force and moment balance on the vehicle.
// These are the fundamental dynamic equations: F = m*a, M = I*α
//
// This is the THIRD equation set in Guiggiani's three-equation structure:
//   1. CONGRUENCE → 2. CONSTITUTIVE → 3. EQUILIBRIUM
//
// Key Sections:
//   - 3.4: Introduction to equilibrium equations
//   - 3.5: Forces acting on the vehicle
//   - 3.6: Explicit form of equilibrium equations
//   - 3.11: Complete vehicle equilibrium (with differential)
//   - Chapter 9: Full 3D equilibrium with roll motion
//
// Guiggiani Notation:
//   X, Y, Z: Forces in body-fixed frame
//   L, M, N: Moments about body axes (roll, pitch, yaw)
//   u, v, w: Linear velocities in body frame
//   p, q, r: Angular velocities (roll rate, pitch rate, yaw rate)

#include "core/math/math_base.h"
#include "core/math/vec3.h"
#include "core/math/mat3.h"

//-------------------------
// Forward Declarations
//-------------------------

typedef struct Vehicle Vehicle;

//-------------------------
// Forces Acting on Vehicle
// (Guiggiani Section 3.5)
//-------------------------

// All forces acting on the vehicle (body frame)
typedef struct VehicleForces {
    //--- External Forces ---
    vde_vec3 weight;             // Weight force (gravity)
    vde_vec3 aero;               // Aerodynamic force
    vde_vec3 tire_forces[4];     // Tire forces at each wheel
    
    //--- Total Force ---
    vde_vec3 total_force;        // Sum of all forces
    
    //--- External Moments ---
    vde_vec3 aero_moment;        // Aerodynamic moment
    vde_vec3 tire_moments[4];    // Tire moments (aligning, overturning)
    
    //--- Total Moment ---
    vde_vec3 total_moment;       // Sum of all moments about CG
} VehicleForces;

//-------------------------
// Load Transfers
// (Guiggiani Section 3.7)
//-------------------------

// Forward declaration - full definition in vehicle_model.h
typedef struct LoadTransfers LoadTransfers;

//-------------------------
// Equations of Motion
// (Guiggiani Section 3.6, 3.11)
//-------------------------

// 6DOF equations of motion in body frame
// Following Guiggiani's notation (Section 3.6)
typedef struct EquationsOfMotion {
    //--- Linear Momentum Balance ---
    // m * (u_dot - v*r + w*q) = X
    // m * (v_dot - w*p + u*r) = Y
    // m * (w_dot - u*q + v*p) = Z
    vde_real force_x;            // X: longitudinal force (N)
    vde_real force_y;            // Y: lateral force (N)
    vde_real force_z;            // Z: vertical force (N)
    
    //--- Angular Momentum Balance ---
    // Ix*p_dot - (Iy-Iz)*q*r = L (roll moment)
    // Iy*q_dot - (Iz-Ix)*r*p = M (pitch moment)
    // Iz*r_dot - (Ix-Iy)*p*q = N (yaw moment)
    vde_real moment_l;           // L: roll moment (Nm)
    vde_real moment_m;           // M: pitch moment (Nm)
    vde_real moment_n;           // N: yaw moment (Nm)
    
    //--- Mass Properties ---
    vde_real mass;               // Total mass (kg)
    vde_mat3 inertia_tensor;     // Inertia tensor (kg*m²)
    
    //--- Accelerations (output after solving) ---
    vde_vec3 linear_accel;       // u_dot, v_dot, w_dot (m/s²)
    vde_vec3 angular_accel;      // p_dot, q_dot, r_dot (rad/s²)
} EquationsOfMotion;

//-------------------------
// Equilibrium Computation API
//-------------------------

// Assemble all forces and moments (Section 3.5)
VDE_API void vehicle_equilibrium_assemble_forces(
    const Vehicle* vehicle,
    VehicleForces* out_forces
);

// Compute load transfers (Section 3.7)
VDE_API void vehicle_equilibrium_compute_load_transfers(
    const Vehicle* vehicle,
    const VehicleForces* forces,
    LoadTransfers* out_transfers
);

// Build equations of motion (Section 3.6)
VDE_API void vehicle_equilibrium_build_equations(
    const Vehicle* vehicle,
    const VehicleForces* forces,
    EquationsOfMotion* out_eom
);

// Solve for accelerations
VDE_API void vehicle_equilibrium_solve_accelerations(
    const EquationsOfMotion* eom,
    vde_vec3* out_linear_accel,
    vde_vec3* out_angular_accel
);

//-------------------------
// Sprung/Unsprung Mass System
// (Guiggiani Section 3.10)
//-------------------------

// Equilibrium for sprung mass (chassis)
VDE_API void vehicle_equilibrium_sprung_mass(
    const Vehicle* vehicle,
    vde_vec3* out_force,
    vde_vec3* out_moment
);

// Equilibrium for each unsprung mass (wheel assembly)
VDE_API void vehicle_equilibrium_unsprung_mass(
    const Vehicle* vehicle,
    int corner_index,
    vde_vec3* out_force,
    vde_vec3* out_moment
);

//-------------------------
// Planar (3DOF) vs Full 3D (6DOF)
//-------------------------

// Simplified planar equilibrium (Chapter 3, 6, 7)
// Only X, Y forces and yaw moment N
typedef struct PlanarEquilibrium {
    vde_real force_x;            // Longitudinal force (N)
    vde_real force_y;            // Lateral force (N)
    vde_real moment_n;           // Yaw moment (Nm)
    vde_real mass;               // Mass (kg)
    vde_real inertia_z;          // Yaw inertia (kg*m²)
    vde_real accel_x;            // u_dot (m/s²)
    vde_real accel_y;            // v_dot (m/s²)
    vde_real accel_r;            // r_dot (rad/s²)
} PlanarEquilibrium;

// Build planar equilibrium equations (for handling analysis)
VDE_API void vehicle_equilibrium_build_planar(
    const Vehicle* vehicle,
    PlanarEquilibrium* out_planar
);

// Full 3D equilibrium with roll (Chapter 9)
VDE_API void vehicle_equilibrium_build_3d(
    const Vehicle* vehicle,
    EquationsOfMotion* out_eom
);

//-------------------------
// Differential Mechanism
// (Guiggiani Section 3.11.4)
//-------------------------

// Equilibrium constraint from differential
typedef enum DifferentialType {
    DIFF_OPEN,                   // Open differential (Chapter 6)
    DIFF_LOCKED,                 // Locked differential (Chapter 7)
    DIFF_LIMITED_SLIP            // Limited slip differential
} DifferentialType;

// Apply differential constraint to wheel torques
VDE_API void vehicle_equilibrium_apply_differential_constraint(
    DifferentialType diff_type,
    vde_real drive_torque,
    vde_real wheel_speeds[4],
    vde_real* out_wheel_torques  // Array of 4 elements
);

//-------------------------
// Validation
//-------------------------

// Verify force/moment balance
VDE_API int vehicle_equilibrium_validate(const Vehicle* vehicle);

// Compute total kinetic energy (for energy conservation checks)
VDE_API vde_real vehicle_equilibrium_compute_kinetic_energy(
    const Vehicle* vehicle
);

//-------------------------
// Key Formulas (from Guiggiani)
//-------------------------

// Section 3.7.1: Longitudinal load transfer
//   ΔFz = (m * ax * h) / l
//   where m=mass, ax=longitudinal accel, h=CG height, l=wheelbase

// Section 3.7.2: Lateral load transfer (front axle)
//   ΔFz_front = (m * ay * h * b/l) / tf
//   where ay=lateral accel, b=CG to rear, tf=front track

// Section 3.11.1: Sprung mass equilibrium (simplified)
//   m * ay = Fy1 + Fy2 + Fy3 + Fy4
//   Iz * r_dot = a*(Fy1+Fy2) - b*(Fy3+Fy4) + tf/2*(Fx2-Fx1+Fx4-Fx3)

