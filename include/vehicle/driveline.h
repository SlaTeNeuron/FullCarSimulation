#pragma once
// Vehicle Dynamics Engine - Driveline System
// Guiggiani Chapter 3, Section 3.11.4: "Principles of Any Differential Mechanism"
//           Chapter 6, Section 6.1: "Open Differential" (road cars)
//           Chapter 7, Section 7.1: "Locked and Limited Slip Differentials" (race cars)
//
// The driveline transmits engine torque to the wheels through transmission,
// final drive, and differential.
//
// Key Sections:
//   - 3.11.4: Differential mechanism principles
//   - 6.1: Open differential behavior (road cars)
//   - 7.1: Locked differential behavior (race cars)
//   - 7.1: Limited slip differential (LSD)
//
// Differential Types:
//   - Open: Tdiff = Tleft + Tright, ωleft + ωright = 2*ωdiff
//   - Locked: ωleft = ωright (rigid connection)
//   - LSD: Torque bias based on speed difference
//
// Key Insight (Guiggiani):
//   Open diff: inner wheel limits traction in corners
//   Locked diff: better traction but handling challenges

#include "core/math/math_base.h"

//-------------------------
// Types
//-------------------------

typedef struct Driveline Driveline;

//-------------------------
// API Functions
//-------------------------

VDE_API Driveline* driveline_create(void);
VDE_API void driveline_destroy(Driveline* driveline);

// Engine control
VDE_API void driveline_set_throttle(Driveline* driveline, vde_real throttle);
VDE_API vde_real driveline_get_throttle(const Driveline* driveline);

// Driveline properties
VDE_API void driveline_set_engine_torque(Driveline* driveline, vde_real torque);
VDE_API void driveline_set_gear_ratio(Driveline* driveline, vde_real ratio);
VDE_API void driveline_set_final_drive(Driveline* driveline, vde_real ratio);

// Compute wheel torque for a driven wheel
VDE_API vde_real driveline_compute_wheel_torque(const Driveline* driveline, int wheel_index);
