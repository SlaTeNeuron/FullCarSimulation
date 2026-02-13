#pragma once
// Vehicle Dynamics Engine - Aerodynamics
// Guiggiani Chapter 7, Section 7.6: "Handling of Formula Cars"
//
// Aerodynamic forces are critical for race car performance, especially
// downforce which increases tire grip at high speeds.
//
// Key Sections:
//   - 7.6: Formula car aerodynamics
//   - 7.6.1: Handling surface with downforce
//   - 7.6.2: MAP (Map of Achievable Performance) with aero
//   - 4.11: Braking performance with downforce
//
// Aerodynamic Forces:
//   Drag = 0.5 * ρ * A * Cd * V²
//   Downforce = 0.5 * ρ * A * Cl * V²
//   where ρ=air density, A=reference area, Cd/Cl=coefficients, V=speed
//
// Key Concepts:
//   - Downforce increases tire vertical loads → more grip
//   - Drag costs top speed but improves braking
//   - Critical for Formula cars and race cars
//   - Minimal effect on road cars

#include "core/math/math_base.h"
#include "core/math/vec3.h"

typedef struct Vehicle Vehicle;

//-------------------------
// Types
//-------------------------

typedef struct Aerodynamics Aerodynamics;

// Aerodynamic force output
typedef struct AeroForces {
    vde_vec3 force;   // Aerodynamic force vector
    vde_vec3 moment;  // Aerodynamic moment vector
} AeroForces;

//-------------------------
// API Functions
//-------------------------

VDE_API Aerodynamics* aerodynamics_create(void);
VDE_API void aerodynamics_destroy(Aerodynamics* aero);

// Aerodynamic properties
VDE_API void aerodynamics_set_drag_coeff(Aerodynamics* aero, vde_real cd);
VDE_API void aerodynamics_set_lift_coeff(Aerodynamics* aero, vde_real cl);
VDE_API void aerodynamics_set_frontal_area(Aerodynamics* aero, vde_real area);

// Compute aerodynamic forces
VDE_API void aerodynamics_compute_forces(const Aerodynamics* aero, const Vehicle* vehicle, AeroForces* out_forces);
