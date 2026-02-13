#pragma once
// Vehicle Dynamics Engine - Track Surface

#include "core/math/math_base.h"
#include "core/math/vec3.h"

//-------------------------
// Types
//-------------------------

typedef struct TrackSurface TrackSurface;

// Surface properties at a point
typedef struct SurfaceProperties {
    vde_real friction_coeff;     // Friction coefficient
    vde_real roughness;          // Surface roughness
    vde_vec3 normal;             // Surface normal vector
    vde_real elevation;          // Elevation at point (m)
} SurfaceProperties;

//-------------------------
// API Functions
//-------------------------

VDE_API TrackSurface* track_surface_create(void);
VDE_API void track_surface_destroy(TrackSurface* surface);

// Query surface properties at a position
VDE_API void track_surface_get_properties(
    const TrackSurface* surface,
    const vde_vec3* position,
    SurfaceProperties* out_props
);

// Get friction coefficient at position
VDE_API vde_real track_surface_get_friction(
    const TrackSurface* surface,
    const vde_vec3* position
);

// Set uniform properties
VDE_API void track_surface_set_uniform_friction(TrackSurface* surface, vde_real friction);

