#pragma once
// Vehicle Dynamics Engine - Steering System
// Guiggiani Chapter 3, Section 3.2.3: "Steering Geometry"
//           Chapter 6, Section 6.18: "Compliant Steering System"
//
// The steering system relates driver input to wheel steer angles,
// typically using Ackermann geometry for kinematic correctness.
//
// Key Sections:
//   - 3.2.3: Steering kinematics and Ackermann geometry
//   - 6.18: Compliant steering effects on handling
//   - 6.18.2: How compliance affects understeer/oversteer
//
// Ackermann Geometry:
//   Ideal geometry for zero-slip turning (low speed)
//   Inner wheel: cot(δi) = l/R + t/(2R)
//   Outer wheel: cot(δo) = l/R - t/(2R)
//   where l=wheelbase, R=turn radius, t=track width
//
// Key Concepts:
//   - Steering ratio: driver input to wheel angle
//   - Ackermann reduces tire scrub in low-speed turns
//   - Steering compliance affects vehicle response

#include "core/math/math_base.h"

//-------------------------
// Types
//-------------------------

typedef struct Steering Steering;

//-------------------------
// API Functions
//-------------------------

VDE_API Steering* steering_create(void);
VDE_API void steering_destroy(Steering* steering);

// Set steering input
VDE_API void steering_set_angle(Steering* steering, vde_real angle_rad);
VDE_API vde_real steering_get_angle(const Steering* steering);

// Steering properties
VDE_API void steering_set_ratio(Steering* steering, vde_real ratio);
VDE_API vde_real steering_get_ratio(const Steering* steering);

// Compute wheel steer angle for a corner (applies Ackermann geometry)
VDE_API vde_real steering_compute_wheel_angle(const Steering* steering, int corner_index);
