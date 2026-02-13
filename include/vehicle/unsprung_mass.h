#pragma once
// Vehicle Dynamics Engine - Unsprung Mass

#include "core/math/math_base.h"
#include "core/math/vec3.h"

//-------------------------
// Types
//-------------------------

typedef struct UnsprungMass UnsprungMass;

//-------------------------
// API Functions
//-------------------------

VDE_API UnsprungMass* unsprung_mass_create(void);
VDE_API void unsprung_mass_destroy(UnsprungMass* um);

// State accessors
VDE_API void unsprung_mass_get_position(const UnsprungMass* um, vde_vec3* out_position);
VDE_API void unsprung_mass_set_position(UnsprungMass* um, const vde_vec3* position);

VDE_API void unsprung_mass_get_velocity(const UnsprungMass* um, vde_vec3* out_velocity);
VDE_API void unsprung_mass_set_velocity(UnsprungMass* um, const vde_vec3* velocity);

// Wheel spin
VDE_API vde_real unsprung_mass_get_wheel_spin_rate(const UnsprungMass* um);
VDE_API void unsprung_mass_set_wheel_spin_rate(UnsprungMass* um, vde_real spin_rate);

// Properties
VDE_API void unsprung_mass_set_mass(UnsprungMass* um, vde_real mass);
VDE_API vde_real unsprung_mass_get_mass(const UnsprungMass* um);

// Force accumulation
VDE_API void unsprung_mass_reset_forces(UnsprungMass* um);
VDE_API void unsprung_mass_apply_force(UnsprungMass* um, const vde_vec3* force);
VDE_API void unsprung_mass_get_accumulated_force(const UnsprungMass* um, vde_vec3* out_force);
