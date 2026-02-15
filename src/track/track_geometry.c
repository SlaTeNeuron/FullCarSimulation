#include "track/track_geometry.h"
#include <stdlib.h>
#include <string.h>

//-------------------------
// Internal State
//-------------------------

struct TrackGeometry {
    TrackSegment* segments;  // Dynamic array of segments
    int num_segments;        // Current number of segments
    int capacity;            // Allocated capacity
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create track geometry
 * 
 * Output:
 *   - Returns pointer to TrackGeometry, or NULL on failure
 */
TrackGeometry* track_geometry_create(void) {
    TrackGeometry* geometry = (TrackGeometry*)malloc(sizeof(TrackGeometry));
    if (!geometry) return NULL;
    
    geometry->segments = NULL;
    geometry->num_segments = 0;
    geometry->capacity = 0;
    
    return geometry;
}

/**
 * Destroy track geometry
 * 
 * Input:
 *   - geometry: Track geometry to destroy (can be NULL)
 */
void track_geometry_destroy(TrackGeometry* geometry) {
    if (!geometry) return;
    
    if (geometry->segments) {
        free(geometry->segments);
    }
    free(geometry);
}

//-------------------------
// Segment Management
//-------------------------

/**
 * Add a track segment
 * 
 * Input:
 *   - geometry: Track geometry (must be non-NULL)
 *   - segment: Segment to add (must be non-NULL)
 * 
 * Functionality:
 *   Adds segment to track, expanding capacity if needed.
 */
void track_geometry_add_segment(TrackGeometry* geometry, const TrackSegment* segment) {
    if (!geometry || !segment) return;
    
    // Expand capacity if needed
    if (geometry->num_segments >= geometry->capacity) {
        int new_capacity = (geometry->capacity == 0) ? 16 : geometry->capacity * 2;
        TrackSegment* new_segments = (TrackSegment*)realloc(
            geometry->segments,
            new_capacity * sizeof(TrackSegment)
        );
        if (!new_segments) return;
        
        geometry->segments = new_segments;
        geometry->capacity = new_capacity;
    }
    
    // Add segment
    geometry->segments[geometry->num_segments] = *segment;
    geometry->num_segments++;
}

/**
 * Get number of segments
 * 
 * Input:
 *   - geometry: Track geometry (must be non-NULL)
 * 
 * Output:
 *   - Returns number of segments
 */
int track_geometry_get_segment_count(const TrackGeometry* geometry) {
    if (!geometry) return 0;
    return geometry->num_segments;
}

//-------------------------
// Queries
//-------------------------

/**
 * Find closest point on track to given position
 * 
 * Input:
 *   - geometry: Track geometry (must be non-NULL)
 *   - position: Query position (must be non-NULL)
 *   - out_closest_point: Output for closest point (can be NULL)
 *   - out_distance: Output for distance (can be NULL)
 * 
 * Output:
 *   - out_closest_point: Filled with closest point if non-NULL
 *   - out_distance: Filled with distance if non-NULL
 * 
 * Functionality:
 *   1. Iterate through all segments
 *   2. Find closest point on each segment
 *   3. Return overall closest point and distance
 */
void track_geometry_get_closest_point(
    const TrackGeometry* geometry,
    const vde_vec3* position,
    vde_vec3* out_closest_point,
    vde_real* out_distance
) {
    if (!geometry || !position) return;
    
    // TODO: Implement closest point search
    // For each segment:
    //   - If straight: project onto line
    //   - If curve: find closest point on arc
    
    if (out_closest_point) {
        *out_closest_point = vde_vec3_zero();
    }
    if (out_distance) {
        *out_distance = (vde_real)0.0;
    }
}

/**
 * Get track centerline tangent at position
 * 
 * Input:
 *   - geometry: Track geometry (must be non-NULL)
 *   - position: Query position (must be non-NULL)
 *   - out_tangent: Output tangent vector (must be non-NULL)
 * 
 * Output:
 *   - out_tangent: Filled with tangent direction
 * 
 * Functionality:
 *   1. Find which segment contains the position
 *   2. Compute tangent for that segment
 *   3. For straight: tangent = (end - start) / length
 *   4. For curve: tangent perpendicular to radius
 */
void track_geometry_get_tangent(
    const TrackGeometry* geometry,
    const vde_vec3* position,
    vde_vec3* out_tangent
) {
    if (!geometry || !position || !out_tangent) return;
    
    // TODO: Implement tangent computation
    
    // Placeholder: forward direction
    *out_tangent = vde_vec3_make(1, 0, 0);
} functions
