#pragma once
// Vehicle Dynamics Engine - Numerical Integrator Base

#include "core/math/math_base.h"

typedef struct EquationsOfMotion EquationsOfMotion;

//-------------------------
// Types
//-------------------------

typedef struct IntegratorBase IntegratorBase;

// State derivative function pointer
typedef void (*DerivativeFunc)(const vde_real* state, vde_real* derivatives, void* user_data);

//-------------------------
// API Functions
//-------------------------

VDE_API IntegratorBase* integrator_create(void);
VDE_API void integrator_destroy(IntegratorBase* integrator);

// Perform integration step
// state: current state vector (in/out)
// n: size of state vector
// dt: time step
// deriv_func: function to compute derivatives
// user_data: user data passed to derivative function
VDE_API void integrator_step(
    IntegratorBase* integrator,
    vde_real* state,
    int n,
    vde_real dt,
    DerivativeFunc deriv_func,
    void* user_data
);