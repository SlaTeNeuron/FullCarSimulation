#include "track/track_surface.h"
#include <stdlib.h>

//-------------------------
// Internal State
//-------------------------

struct TrackSurface {
    vde_real uniform_friction;  // Uniform friction coefficient
    vde_real uniform_roughness; // Uniform roughness
    void* friction_map;         // Optional friction map
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create track surface
 * 
 * Output:
 *   - Returns pointer to TrackSurface, or NULL on failure
 */
TrackSurface* track_surface_create(void) {
    TrackSurface* surface = (TrackSurface*)malloc(sizeof(TrackSurface));
    if (!surface) return NULL;
    
    surface->uniform_friction = (vde_real)1.0; // Dry asphalt
    surface->uniform_roughness = (vde_real)0.01;
    surface->friction_map = NULL;
    
    return surface;
}

/**
 * Destroy track surface
 * 
 * Input:
 *   - surface: Track surface to destroy (can be NULL)
 */
void track_surface_destroy(TrackSurface* surface) {
    if (!surface) return;
    
    // TODO: Destroy friction map if allocated
    
    free(surface);
}

//-------------------------
// Uniform Properties
//-------------------------

/**
 * Set uniform friction coefficient
 * 
 * Input:
 *   - surface: Track surface (must be non-NULL)
 *   - friction: Friction coefficient (typically 0.7-1.2)
 */
void track_surface_set_uniform_friction(TrackSurface* surface, vde_real friction) {
    if (!surface) return;
    surface->uniform_friction = friction;
}

//-------------------------
// Queries
//-------------------------

/**
 * Get surface properties at a position
 * 
 * Guiggiani Reference: Chapter 5 (Track characteristics)
 * 
 * Input:
 *   - surface: Track surface (must be non-NULL)
 *   - position: Query position (must be non-NULL)
 *   - out_props: Output properties (must be non-NULL)
 * 
 * Output:
 *   - out_props: Filled with surface properties at position
 * 
 * Functionality:
 *   1. Check if friction map exists
 *   2. If yes, sample from map
 *   3. If no, use uniform properties
 *   4. Compute surface normal (typically vertical)
 *   5. Get elevation from terrain heightmap if available
 */
void track_surface_get_properties(
    const TrackSurface* surface,
    const vde_vec3* position,
    SurfaceProperties* out_props
) {
    if (!surface || !position || !out_props) return;
    
    // TODO: Sample from friction map if available
    // For now, use uniform properties
    
    out_props->friction_coeff = surface->uniform_friction;
    out_props->roughness = surface->uniform_roughness;
    out_props->normal = vde_vec3_make(0, 0, 1); // Flat surface
    out_props->elevation = (vde_real)0.0;
}

/**
 * Get friction coefficient at position
 * 
 * Input:
 *   - surface: Track surface (must be non-NULL)
 *   - position: Query position (must be non-NULL)
 * 
 * Output:
 *   - Returns friction coefficient at position
 * 
 * Functionality:
 *   Simplified version of get_properties that returns only friction.
 */
vde_real track_surface_get_friction(
    const TrackSurface* surface,
    const vde_vec3* position
) {
    if (!surface || !position) return (vde_real)1.0;
    
    // TODO: Sample from friction map if available
    
    return surface->uniform_friction;
}
