#pragma once
// Vehicle Dynamics Engine - Runge-Kutta 4th Order Integrator

#include "core/math/math_base.h"
#include "integrator_base.h"

//-------------------------
// Types
//-------------------------

typedef struct RungeKutta4 RungeKutta4;

//-------------------------
// API Functions
//-------------------------

VDE_API RungeKutta4* rk4_create(void);
VDE_API void rk4_destroy(RungeKutta4* integrator);

// Perform RK4 integration step
VDE_API void rk4_step(
    RungeKutta4* integrator,
    vde_real* state,
    int n,
    vde_real dt,
    DerivativeFunc deriv_func,
    void* user_data
);