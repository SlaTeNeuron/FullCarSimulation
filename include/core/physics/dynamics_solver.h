#pragma once
// Vehicle Dynamics Engine - Dynamics Solver

#include "core/math/math_base.h"

typedef struct Vehicle Vehicle;
typedef struct EquationsOfMotion EquationsOfMotion;

//-------------------------
// Types
//-------------------------

typedef struct DynamicsSolver DynamicsSolver;

//-------------------------
// API Functions
//-------------------------

VDE_API DynamicsSolver* dynamics_solver_create(void);
VDE_API void dynamics_solver_destroy(DynamicsSolver* solver);

// Solve vehicle dynamics for one timestep
VDE_API void dynamics_solver_solve(
    DynamicsSolver* solver,
    Vehicle* vehicle,
    vde_real dt
);

// Build and solve equations of motion
VDE_API void dynamics_solver_build_and_solve(
    DynamicsSolver* solver,
    const Vehicle* vehicle,
    EquationsOfMotion* eom
);