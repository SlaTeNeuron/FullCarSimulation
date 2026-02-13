#pragma once
// Vehicle Dynamics Engine - Constraint Solver

#include "core/math/math_base.h"
#include "core/math/vec3.h"

//-------------------------
// Types
//-------------------------

typedef struct Constraint Constraint;
typedef struct ConstraintSolver ConstraintSolver;

// Constraint types
typedef enum ConstraintType {
    CONSTRAINT_DISTANCE,      // Fixed distance between two points
    CONSTRAINT_HINGE,         // Hinge joint (1 rotational DOF)
    CONSTRAINT_SPHERICAL,     // Ball joint (3 rotational DOFs)
    CONSTRAINT_GROUND         // Ground contact constraint
} ConstraintType;

//-------------------------
// API Functions
//-------------------------

VDE_API ConstraintSolver* constraint_solver_create(void);
VDE_API void constraint_solver_destroy(ConstraintSolver* solver);

// Add constraint
VDE_API Constraint* constraint_create(ConstraintType type);
VDE_API void constraint_destroy(Constraint* constraint);

// Solve constraints
VDE_API void constraint_solver_solve(
    ConstraintSolver* solver,
    Constraint** constraints,
    int num_constraints,
    vde_real dt
);