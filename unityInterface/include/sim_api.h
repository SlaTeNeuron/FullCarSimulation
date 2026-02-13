#pragma once
// Vehicle Dynamics Engine - Unity/External Interface

#include "math/math_base.h"

//-------------------------
// Forward Declarations
//-------------------------

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Sim Sim;
typedef struct SimState SimState;

//-------------------------
// API Functions
//-------------------------

Sim* sim_create(vde_real timestep);
void sim_destroy(Sim*);
void sim_step(Sim*);
void sim_set_inputs(Sim*, vde_real throttle, vde_real brake, vde_real steer);
void sim_get_state(Sim*, SimState* out);

#ifdef __cplusplus
}
#endif