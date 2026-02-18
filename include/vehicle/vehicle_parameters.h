#pragma once
// Vehicle Dynamics Engine - Vehicle Parameters

#include "core/math/math_base.h"
#include "core/math/vec3.h"
#include "core/math/mat3.h"

//-------------------------
// Types
//-------------------------

typedef struct VehicleParameters VehicleParameters;

// Mass properties
typedef struct MassProperties {
    vde_real mass;               // Total mass (kg)
    vde_mat3 inertia_tensor;     // Inertia tensor (kg*m^2)
    vde_vec3 cg_position;        // Center of gravity position (m)
} MassProperties;

// Suspension geometry
typedef struct SuspensionGeometry {
    vde_real wheelbase;          // Wheelbase (m)
    vde_real track_front;        // Front track width (m)
    vde_real track_rear;         // Rear track width (m)
    vde_real cg_height;          // CG height (m)
} SuspensionGeometry;

// Tire parameters
typedef struct TireParameters {
    vde_real radius;             // Tire radius (m)
    vde_real width;              // Tire width (m)
    vde_real rolling_resistance; // Rolling resistance coefficient
} TireParameters;

//-------------------------
// API Functions
//-------------------------

VDE_API VehicleParameters* vehicle_params_create(void);
VDE_API void vehicle_params_destroy(VehicleParameters* params);

// Load from file
VDE_API int vehicle_params_load_from_file(VehicleParameters* params, const char* filename);
VDE_API int vehicle_params_save_to_file(const VehicleParameters* params, const char* filename);

// Get/set parameter groups
VDE_API void vehicle_params_set_mass_properties(VehicleParameters* params, const MassProperties* mass);
VDE_API void vehicle_params_get_mass_properties(const VehicleParameters* params, MassProperties* out_mass);

VDE_API void vehicle_params_set_suspension_geometry(VehicleParameters* params, const SuspensionGeometry* geom);
VDE_API void vehicle_params_get_suspension_geometry(const VehicleParameters* params, SuspensionGeometry* out_geom);

VDE_API void vehicle_params_set_tire_parameters(VehicleParameters* params, const TireParameters* tire);
VDE_API void vehicle_params_get_tire_parameters(const VehicleParameters* params, TireParameters* out_tire);