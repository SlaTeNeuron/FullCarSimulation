#pragma once
// Vehicle Dynamics Engine - Semi-Implicit Euler Integrator

#include "core/math/math_base.h"
#include "integrator_base.h"

//-------------------------
// Types
//-------------------------

typedef struct SemiImplicitEuler SemiImplicitEuler;

//-------------------------
// API Functions
//-------------------------

VDE_API SemiImplicitEuler* semi_implicit_euler_create(void);
VDE_API void semi_implicit_euler_destroy(SemiImplicitEuler* integrator);

// Perform integration step (velocity-first, then position)
VDE_API void semi_implicit_euler_step(
    SemiImplicitEuler* integrator,
    vde_real* state,
    int n,
    vde_real dt,
    DerivativeFunc deriv_func,
    void* user_data
);