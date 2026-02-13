#pragma once
// Vehicle Dynamics Engine - Equations of Motion
// Guiggiani Chapter 3, Sections 3.4-3.6: "Vehicle Equilibrium Equations"
//           Chapter 9, Section 9.6: "Full 3D Vehicle Dynamics"
//
// This module builds and solves the vehicle EQUILIBRIUM equations - the
// third component of Guiggiani's three-equation structure.
//
// Guiggiani's Approach (Section 3.12):
//   1. CONGRUENCE: Kinematic relationships
//   2. CONSTITUTIVE: Component behavior (forces from slips, deflections)
//   3. EQUILIBRIUM: Force and moment balance (THIS MODULE)
//
// Key Sections:
//   - 3.4: Introduction to equilibrium equations
//   - 3.5: Forces acting on the vehicle
//   - 3.6: Explicit equilibrium equations
//   - 3.11: Complete vehicle equilibrium with differential
//   - 9.6: Full 6DOF dynamics with roll motion
//
// Equations (Section 3.6 in body-fixed frame):
//   Linear momentum:
//     m * (u_dot - v*r + w*q) = X
//     m * (v_dot - w*p + u*r) = Y
//     m * (w_dot - u*q + v*p) = Z
//
//   Angular momentum:
//     Ix*p_dot - (Iy-Iz)*q*r = L
//     Iy*q_dot - (Iz-Ix)*r*p = M
//     Iz*r_dot - (Ix-Iy)*p*q = N
//
// where u,v,w = linear velocities, p,q,r = angular velocities
//       X,Y,Z = forces, L,M,N = moments, I = inertias

#include "core/math/math_base.h"
#include "core/math/vec3.h"
#include "core/math/mat3.h"

typedef struct Vehicle Vehicle;

//-------------------------
// Types
//-------------------------

typedef struct EquationsOfMotion EquationsOfMotion;

// Generalized state vector (position, velocity, etc.)
typedef struct GeneralizedState {
    vde_real* q;          // Generalized coordinates
    vde_real* q_dot;      // Generalized velocities
    int size;             // Number of DOFs
} GeneralizedState;

//-------------------------
// API Functions
//-------------------------

VDE_API EquationsOfMotion* eom_create(int num_dofs);
VDE_API void eom_destroy(EquationsOfMotion* eom);

// Build equations from vehicle state
VDE_API void eom_build_from_vehicle(EquationsOfMotion* eom, const Vehicle* vehicle);

// Access mass matrix and force vector
VDE_API void eom_get_mass_matrix(const EquationsOfMotion* eom, vde_real* out_matrix);
VDE_API void eom_get_force_vector(const EquationsOfMotion* eom, vde_real* out_forces);

// Solve for accelerations: M * q_ddot = Q
VDE_API void eom_solve_accelerations(const EquationsOfMotion* eom, vde_real* out_accelerations);
