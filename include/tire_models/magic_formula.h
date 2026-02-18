#pragma once
// Vehicle Dynamics Engine - Pacejka Magic Formula Tire Model
// Guiggiani Chapter 2, Section 2.10: "Magic Formula"
//
// The Magic Formula is an empirical tire model developed by Hans Pacejka.
// It provides accurate curve-fitting of experimental tire data.
//
// Key Sections:
//   - 2.10: Magic Formula introduction and equations
//   - 2.9: Tire testing (provides data for Magic Formula)
//
// Magic Formula Structure (Section 2.10):
//   y(x) = D * sin[C * arctan{B*x - E*(B*x - arctan(B*x))}]
//   where:
//     B = stiffness factor
//     C = shape factor
//     D = peak factor (peak value)
//     E = curvature factor
//     x = slip variable (σ for longitudinal, α for lateral)
//     y = force or moment
//
// Advantages:
//   - Fits experimental data very accurately
//   - Computationally efficient
//   - Industry standard for vehicle dynamics
//
// Note: This is a CONSTITUTIVE model - part of Guiggiani's
//       three-equation structure (Congruence → Constitutive → Equilibrium)

#include "core/math/math_base.h"
#include "core/math/vec3.h"
#include "../vehicle/tire.h"

//-------------------------
// Types
//-------------------------

typedef struct MagicFormulaTireModel MagicFormulaTireModel;

// Magic Formula coefficients (simplified MF5.2)
typedef struct MagicFormulaParams {
    // Longitudinal coefficients
    vde_real b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10;
    // Lateral coefficients
    vde_real a0, a1, a2, a3, a4, a5, a6, a7, a8;
    // Vertical stiffness
    vde_real vertical_stiffness;
} MagicFormulaParams;

//-------------------------
// API Functions
//-------------------------

VDE_API MagicFormulaTireModel* magic_formula_create(void);
VDE_API void magic_formula_destroy(MagicFormulaTireModel* model);

// Set parameters
VDE_API void magic_formula_set_params(MagicFormulaTireModel* model, const MagicFormulaParams* params);
VDE_API void magic_formula_get_params(const MagicFormulaTireModel* model, MagicFormulaParams* out_params);

// Compute forces using Magic Formula
VDE_API void magic_formula_compute_forces(
    const MagicFormulaTireModel* model,
    const TireState* state,
    TireForces* out_forces
);