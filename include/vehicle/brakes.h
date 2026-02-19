#pragma once
// Vehicle Dynamics Engine - Brake System
// Guiggiani Chapter 4: "Braking Performance"
//
// The brake system is a critical component for vehicle safety and performance.
// Guiggiani dedicates an entire chapter to analyzing optimal braking.
//
// Key Sections:
//   - 4.3: Equilibrium equations during braking
//   - 4.4: Longitudinal load transfer under braking
//   - 4.5: Maximum deceleration conditions
//   - 4.6: Optimal brake balance for max deceleration
//   - 4.7: All possible braking combinations
//   - 4.11: Formula car braking with downforce
//
// Key Concepts:
//   - Brake balance: distribution of braking force front/rear
//   - Load transfer increases front tire grip during braking
//   - Optimal balance depends on CG height, wheelbase, grip
//
// Optimal Brake Balance (Section 4.6):
//   φ = b/l + (μ*h/l)   (front brake force fraction)
//   where b=CG to rear, l=wheelbase, h=CG height, μ=friction

#include "core/math/math_base.h"

//-------------------------
// Types
//-------------------------

typedef struct Brakes Brakes;

//-------------------------
// API Functions
//-------------------------

VDE_API Brakes* brakes_create(void);
VDE_API void brakes_destroy(Brakes* brakes);

// Set brake input (0 to 1)
VDE_API void brakes_set_pressure(Brakes* brakes, vde_real pressure);
VDE_API vde_real brakes_get_pressure(const Brakes* brakes);

// Brake properties — separate front/rear physical limits
VDE_API void brakes_set_max_torque_front(Brakes* brakes, vde_real max_torque);
VDE_API vde_real brakes_get_max_torque_front(const Brakes* brakes);
VDE_API void brakes_set_max_torque_rear(Brakes* brakes, vde_real max_torque);
VDE_API vde_real brakes_get_max_torque_rear(const Brakes* brakes);

// Convenience: set the same torque limit for all four wheels
VDE_API void brakes_set_max_torque(Brakes* brakes, vde_real max_torque);
VDE_API vde_real brakes_get_max_torque(const Brakes* brakes);

// Front/rear balance: fraction of braking force on front axle [0, 1]
// Guiggiani Sec. 4.6: optimal φ = c/L + μ·h/L
VDE_API void brakes_set_brake_balance(Brakes* brakes, vde_real balance);
VDE_API vde_real brakes_get_brake_balance(const Brakes* brakes);

// Compute brake torque for a specific wheel (applies balance internally)
VDE_API vde_real brakes_compute_torque(const Brakes* brakes, int wheel_index);
