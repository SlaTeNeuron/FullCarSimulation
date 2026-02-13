#pragma once
// Vehicle Dynamics Engine - Vehicle Model

#include "core/math/math_base.h"
#include "core/math/vec3.h"
#include "core/math/quat.h"
#include "core/math/frames.h"

//-------------------------
// Forward Declarations
//-------------------------

typedef struct SprungMass SprungMass;
typedef struct UnsprungMass UnsprungMass;
typedef struct Wheel Wheel;
typedef struct Tire Tire;
typedef struct Suspension Suspension;
typedef struct Steering Steering;
typedef struct Driveline Driveline;
typedef struct Brakes Brakes;
typedef struct Aerodynamics Aerodynamics;
typedef struct VehicleParameters VehicleParameters;

//-------------------------
// Types
//-------------------------

typedef struct Vehicle Vehicle;

//-------------------------
// API Functions
//-------------------------

VDE_API Vehicle* vehicle_create(void);
VDE_API void vehicle_destroy(Vehicle* v);

// Control interface
VDE_API void vehicle_set_inputs(Vehicle* v, vde_real throttle, vde_real brake, vde_real steer);
VDE_API void vehicle_step(Vehicle* v, vde_real dt);

// Component access
VDE_API SprungMass* vehicle_get_sprung_mass(Vehicle* v);
VDE_API UnsprungMass* vehicle_get_unsprung_mass(Vehicle* v, int corner_index);
VDE_API Wheel* vehicle_get_wheel(Vehicle* v, int corner_index);
VDE_API Tire* vehicle_get_tire(Vehicle* v, int corner_index);
VDE_API Suspension* vehicle_get_suspension(Vehicle* v);
VDE_API Steering* vehicle_get_steering(Vehicle* v);
VDE_API Driveline* vehicle_get_driveline(Vehicle* v);
VDE_API Brakes* vehicle_get_brakes(Vehicle* v);
VDE_API Aerodynamics* vehicle_get_aerodynamics(Vehicle* v);

// Update functions
VDE_API void vehicle_update_kinematics(Vehicle* v, vde_real dt);
VDE_API void vehicle_compute_forces(Vehicle* v);

// Telemetry
VDE_API void vehicle_write_telemetry(Vehicle* v, vde_real* buffer, int len);
