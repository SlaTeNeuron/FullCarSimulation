#pragma once
// Vehicle Dynamics Engine - Tire Component
// Guiggiani Chapter 2: "Mechanics of the Wheel with Tire"
//           Chapter 10: "Tire Models" (Advanced theory)
//
// The tire is the critical vehicle component - it generates the grip forces
// that enable acceleration, braking, and cornering. Guiggiani introduces
// tires FIRST before the vehicle model.
//
// Key Concepts:
//   - Tire as constitutive element: Forces = f(slips, load)
//   - Contact patch and footprint forces (Section 2.4-2.5)
//   - Grip forces vs. tire slips (Section 2.8)
//   - Tire testing and characterization (Section 2.9)
//
// Key Sections:
//   - 2.6: Tire global mechanical behavior
//   - 2.7: Tire slips (σ, α, ϕ)
//   - 2.8: Grip forces and tire slips
//   - 2.9: Tire testing (pure longitudinal, pure lateral)
//   - 2.10: Magic Formula (empirical model)
//   - Chapter 10: Brush model (theoretical/physical model)
//
// Constitutive Relation:
//   [Fx, Fy, Fz, Mx, My, Mz] = tire_model(σ, α, ϕ, Fz, γ, p, ...)
//   where σ=long. slip, α=slip angle, ϕ=spin, γ=camber, p=pressure

#include "core/math/math_base.h"
#include "core/math/vec3.h"

//-------------------------
// Types
//-------------------------

typedef struct Tire Tire;

// Tire forces output structure
typedef struct TireForces {
    vde_vec3 force;          // Force in tire frame (longitudinal, lateral, vertical)
    vde_vec3 moment;         // Moment about tire contact patch
    vde_real slip_ratio;     // Computed slip ratio
    vde_real slip_angle;     // Computed slip angle (rad)
} TireForces;

// Tire state inputs
typedef struct TireState {
    vde_real normal_load;    // Normal load (N)
    vde_real slip_ratio;     // Longitudinal slip ratio
    vde_real slip_angle;     // Lateral slip angle (rad)
    vde_real camber_angle;   // Camber angle (rad)
    vde_real angular_velocity; // Wheel angular velocity (rad/s)
} TireState;

//-------------------------
// API Functions
//-------------------------

VDE_API Tire* tire_create(void);
VDE_API void tire_destroy(Tire* tire);

// Compute tire forces given current state
VDE_API void tire_compute_forces(Tire* tire, const TireState* state, TireForces* out_forces);

// Set tire properties
VDE_API void tire_set_radius(Tire* tire, vde_real radius);
VDE_API vde_real tire_get_radius(const Tire* tire);
