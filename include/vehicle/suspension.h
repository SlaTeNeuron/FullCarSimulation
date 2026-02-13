#pragma once
// Vehicle Dynamics Engine - Suspension System
// Guiggiani Chapter 3, Section 3.8: "Suspension First-Order Analysis"
//           Chapter 8: "Ride Comfort and Road Holding"
//
// Suspension constitutive equations relate deflections and velocities
// to forces and moments.
//
// Key Sections:
//   - 3.8.5: Roll and vertical stiffnesses
//   - 3.8.8: Suspension kinematics
//   - 3.8.12: Lateral load transfer expressions
//   - 8.2: Quarter car model (sprung/unsprung dynamics)
//   - 8.2.1: Inerter as spring softener
//   - 8.3: Shock absorber tuning (comfort vs. road holding)
//   - 8.5: Full vehicle ride modes (bounce, pitch, roll)
//
// Constitutive Laws:
//   Fspring = k * Δz        (spring stiffness)
//   Fdamper = c * Δz_dot    (damping coefficient)

#include "core/math/math_base.h"
#include "core/math/vec3.h"

//-------------------------
// Types
//-------------------------

typedef struct Suspension Suspension;

// Suspension corner state
typedef struct SuspensionCorner {
    vde_real spring_deflection;  // Spring compression/extension (m)
    vde_real spring_rate;        // Spring stiffness (N/m)
    vde_real damper_velocity;    // Damper velocity (m/s)
    vde_real damper_coeff;       // Damping coefficient (N*s/m)
    vde_real rest_length;        // Spring rest length (m)
} SuspensionCorner;

// Suspension force output
typedef struct SuspensionForce {
    vde_vec3 force;              // Force vector
    vde_vec3 moment;             // Moment vector
} SuspensionForce;

//-------------------------
// API Functions
//-------------------------

VDE_API Suspension* suspension_create(void);
VDE_API void suspension_destroy(Suspension* susp);

// Compute suspension forces for a corner
VDE_API void suspension_compute_forces(const Suspension* susp, int corner_index, SuspensionForce* out_force);

// Get/set corner properties
VDE_API void suspension_set_corner(Suspension* susp, int corner_index, const SuspensionCorner* corner);
VDE_API void suspension_get_corner(const Suspension* susp, int corner_index, SuspensionCorner* out_corner);

// Update suspension state
VDE_API void suspension_update(Suspension* susp, int corner_index, vde_real deflection, vde_real velocity);
