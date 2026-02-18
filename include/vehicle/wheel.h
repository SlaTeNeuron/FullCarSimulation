#pragma once
// Vehicle Dynamics Engine - Wheel Component
// Guiggiani Chapter 2, Section 2.11: "Mechanics of Wheels with Tire"
//           Chapter 3, Section 3.2.7: "Tire Kinematics (Tire Slips)"
//
// The wheel represents the rotating wheel assembly, responsible for:
//   - Wheel angular velocity (spin)
//   - Computing tire slips from kinematics (congruence equations)
//   - Wheel inertia and dynamics
//
// Key Sections:
//   - 2.7: Tire slips (longitudinal σ, lateral α, spin ϕ)
//   - 2.11: Wheel dynamics and torque balance
//   - 3.2.7: Computing tire slips from vehicle motion
//
// Slip Definitions (Section 2.7):
//   σ = (ω*re - Vx) / |Vx|     (longitudinal slip ratio)
//   tan(α) = Vy / |Vx|         (slip angle)
//   where ω is wheel speed, re is effective radius, Vx/Vy are velocities

#include "core/math/math_base.h"
#include "core/math/vec3.h"

typedef struct Vehicle Vehicle;

//-------------------------
// Types
//-------------------------

typedef struct Wheel Wheel;

//-------------------------
// API Functions
//-------------------------

VDE_API Wheel* wheel_create(void);
VDE_API void wheel_destroy(Wheel* wheel);

// Update wheel state
VDE_API void wheel_set_angular_velocity(Wheel* wheel, vde_real omega);
VDE_API vde_real wheel_get_angular_velocity(const Wheel* wheel);

// Wheel properties
VDE_API void wheel_set_radius(Wheel* wheel, vde_real radius);
VDE_API vde_real wheel_get_radius(const Wheel* wheel);
VDE_API void wheel_set_inertia(Wheel* wheel, vde_real inertia);
VDE_API vde_real wheel_get_inertia(const Wheel* wheel);

// Compute slip quantities
VDE_API vde_real wheel_compute_slip_ratio(const Wheel* wheel, const Vehicle* vehicle, int wheel_index);
VDE_API vde_real wheel_compute_slip_angle(const Wheel* wheel, const Vehicle* vehicle, int wheel_index);
