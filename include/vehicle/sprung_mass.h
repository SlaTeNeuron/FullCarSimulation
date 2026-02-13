#pragma once
// Vehicle Dynamics Engine - Sprung Mass (Chassis)
// Guiggiani Chapter 3, Section 3.10: "Sprung and Unsprung Masses"
//           Chapter 9: "Handling with Roll Motion" (Full 3D dynamics)
//
// The sprung mass represents the vehicle chassis/body - the portion of
// the vehicle supported by the suspension springs.
//
// Key Sections:
//   - 3.10: Sprung/unsprung mass decomposition
//   - 9.2: 3D orientation (yaw, pitch, roll)
//   - 9.3-9.4: Angular velocity and acceleration
//   - 9.6: Full 3D equilibrium equations with roll
//
// State Variables:
//   - Position and orientation (vde_frame)
//   - Linear and angular velocities
//   - Mass and inertia tensor

#include "core/math/math_base.h"
#include "core/math/vec3.h"
#include "core/math/quat.h"
#include "core/math/mat3.h"
#include "core/math/frames.h"

//-------------------------
// Types
//-------------------------

typedef struct SprungMass SprungMass;

//-------------------------
// API Functions
//-------------------------

VDE_API SprungMass* sprung_mass_create(void);
VDE_API void sprung_mass_destroy(SprungMass* sm);

// State accessors
VDE_API void sprung_mass_get_frame(const SprungMass* sm, vde_frame* out_frame);
VDE_API void sprung_mass_set_frame(SprungMass* sm, const vde_frame* frame);

VDE_API void sprung_mass_get_velocity(const SprungMass* sm, vde_vec3* out_linear, vde_vec3* out_angular);
VDE_API void sprung_mass_set_velocity(SprungMass* sm, const vde_vec3* linear, const vde_vec3* angular);

// Properties
VDE_API void sprung_mass_set_mass(SprungMass* sm, vde_real mass);
VDE_API vde_real sprung_mass_get_mass(const SprungMass* sm);

VDE_API void sprung_mass_set_inertia(SprungMass* sm, const vde_mat3* inertia);
VDE_API void sprung_mass_get_inertia(const SprungMass* sm, vde_mat3* out_inertia);

// Force accumulation
VDE_API void sprung_mass_reset_forces(SprungMass* sm);
VDE_API void sprung_mass_apply_force(SprungMass* sm, const vde_vec3* force, const vde_vec3* point_body);
VDE_API void sprung_mass_apply_moment(SprungMass* sm, const vde_vec3* moment);

VDE_API void sprung_mass_get_accumulated_forces(const SprungMass* sm, vde_vec3* out_force, vde_vec3* out_moment);
