#pragma once
// Vehicle Dynamics Engine - Tire Utilities
// Guiggiani Chapter 2, Section 2.7: "Tire Slips"
//           Chapter 3, Section 3.2.7: "Tire Kinematics"
//
// Utility functions for computing tire slips and transforming forces.
// These implement the kinematic relationships (CONGRUENCE equations)
// that connect vehicle motion to tire behavior.
//
// Key Sections:
//   - 2.7.1: Rolling velocity definition
//   - 2.7.2: Longitudinal slip ratio
//   - 2.7.3: Slip angle
//   - 3.2.7: Computing tire slips from vehicle kinematics
//
// Slip Definitions (Section 2.7):
//   Longitudinal slip ratio:
//     σ = (ω*re - Vx) / |Vx|
//     where ω=wheel angular velocity, re=effective radius, Vx=velocity
//
//   Slip angle:
//     tan(α) = Vy / |Vx|
//     where Vy=lateral velocity, Vx=longitudinal velocity
//
// Sign Conventions (CRITICAL - Guiggiani is very specific):
//   - Positive σ: wheel faster than vehicle (driving)
//   - Negative σ: wheel slower than vehicle (braking)
//   - α: angle between wheel heading and velocity direction

#include "core/math/math_base.h"
#include "core/math/vec3.h"

//-------------------------
// Utility Functions
//-------------------------

// Compute slip ratio from wheel and vehicle velocities
VDE_API vde_real tire_util_compute_slip_ratio(
    vde_real wheel_angular_velocity,
    vde_real wheel_radius,
    vde_real vehicle_velocity
);

// Compute slip angle from velocity components
VDE_API vde_real tire_util_compute_slip_angle(
    vde_real longitudinal_velocity,
    vde_real lateral_velocity
);

// Transform forces from tire to world frame
VDE_API void tire_util_transform_forces(
    const vde_vec3* tire_force,
    vde_real steer_angle,
    vde_real camber_angle,
    vde_vec3* out_world_force
);

// Compute contact patch velocity
VDE_API void tire_util_compute_contact_velocity(
    const vde_vec3* wheel_center_velocity,
    vde_real wheel_radius,
    vde_real angular_velocity,
    vde_vec3* out_contact_velocity
);