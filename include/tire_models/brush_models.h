#pragma once
// Vehicle Dynamics Engine - Brush Tire Model
// Guiggiani Chapter 10: "Tire Models"
//
// The brush model is a PHYSICAL tire model based on contact patch theory.
// Unlike the empirical Magic Formula, the brush model derives forces from
// first principles of adhesion and sliding in the contact patch.
//
// Key Sections:
//   - 10.1: Brush model definition
//   - 10.2: Contact patch geometry
//   - 10.3: Adhesion and sliding zones
//   - 10.4: Pure longitudinal slip
//   - 10.5: Pure lateral slip
//   - 10.6: Combined slip conditions
//   - 10.7: Spinslip (camber effects)
//   - 10.8: Transient tire behavior
//
// Physical Basis:
//   - Contact patch divided into adhesion and sliding zones
//   - Bristle deflections in contact patch
//   - Transition from sticking to sliding
//   - Pressure distribution effects
//
// Advantages:
//   - Physical insight into tire behavior
//   - Can predict effects of design changes
//   - Handles transient behavior naturally
//
// Note: More complex than Magic Formula but physically grounded.
//       This is a CONSTITUTIVE model in Guiggiani's framework.

#include "core/math/math_base.h"
#include "core/math/vec3.h"
#include "../vehicle/tire.h"

//-------------------------
// Types
//-------------------------

typedef struct BrushTireModel BrushTireModel;

// Brush model parameters
typedef struct BrushTireParams {
    vde_real longitudinal_stiffness;  // Longitudinal stiffness (N)
    vde_real lateral_stiffness;       // Lateral stiffness (N)
    vde_real friction_coeff;          // Friction coefficient
    vde_real contact_patch_length;    // Contact patch length (m)
    vde_real contact_patch_width;     // Contact patch width (m)
} BrushTireParams;

//-------------------------
// API Functions
//-------------------------

VDE_API BrushTireModel* brush_tire_create(void);
VDE_API void brush_tire_destroy(BrushTireModel* model);

// Set parameters
VDE_API void brush_tire_set_params(BrushTireModel* model, const BrushTireParams* params);
VDE_API void brush_tire_get_params(const BrushTireModel* model, BrushTireParams* out_params);

// Compute forces using brush model
VDE_API void brush_tire_compute_forces(
    const BrushTireModel* model,
    const TireState* state,
    TireForces* out_forces
);