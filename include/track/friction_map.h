#pragma once
// Vehicle Dynamics Engine - Friction Map

#include "core/math/math_base.h"
#include "core/math/vec3.h"

//-------------------------
// Types
//-------------------------

typedef struct FrictionMap FrictionMap;

//-------------------------
// API Functions
//-------------------------

VDE_API FrictionMap* friction_map_create(int width, int height);
VDE_API void friction_map_destroy(FrictionMap* map);

// Set/get friction at grid position
VDE_API void friction_map_set_value(FrictionMap* map, int x, int y, vde_real friction);
VDE_API vde_real friction_map_get_value(const FrictionMap* map, int x, int y);

// Sample friction at world position (with interpolation)
VDE_API vde_real friction_map_sample(
    const FrictionMap* map,
    const vde_vec3* world_position
);

// Set map bounds
VDE_API void friction_map_set_bounds(
    FrictionMap* map,
    vde_real min_x, vde_real max_x,
    vde_real min_y, vde_real max_y
);