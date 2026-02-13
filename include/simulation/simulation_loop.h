#pragma once
// Vehicle Dynamics Engine - Simulation Loop

#include "core/math/math_base.h"
#include "simulation_config.h"

typedef struct Vehicle Vehicle;
typedef struct IntegratorBase IntegratorBase;

//-------------------------
// Types
//-------------------------

typedef struct SimulationLoop SimulationLoop;

//-------------------------
// API Functions
//-------------------------

VDE_API SimulationLoop* simulation_loop_create(void);
VDE_API void simulation_loop_destroy(SimulationLoop* loop);

// Set components
VDE_API void simulation_loop_set_vehicle(SimulationLoop* loop, Vehicle* vehicle);
VDE_API void simulation_loop_set_integrator(SimulationLoop* loop, IntegratorBase* integrator);

// Run simulation
VDE_API void simulation_loop_step(SimulationLoop* loop, vde_real dt);
VDE_API void simulation_loop_run(SimulationLoop* loop, vde_real duration);

// Get current time
VDE_API vde_real simulation_loop_get_time(const SimulationLoop* loop);
