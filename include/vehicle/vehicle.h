#pragma once
// Vehicle Dynamics Engine - Vehicle Model

#include "math/math_base.h"
#include "math/vec3.h"
#include "math/quat.h"
#include "math/frames.h"

typedef struct Vehicle Vehicle;

//-------------------------
// API Functions
//-------------------------

Vehicle* vehicle_create();
void vehicle_destroy(Vehicle* v);

/* Control interface */
void vehicle_set_inputs(Vehicle* v, vde_real throttle, vde_real brake, vde_real steer);
void vehicle_step(Vehicle* v, vde_real dt);

/* Telemetry: populate buffer with [position_x, position_y, velocity, yaw] */
void vehicle_write_telemetry(Vehicle* v, vde_real* buffer, int len);