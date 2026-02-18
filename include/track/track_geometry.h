#pragma once
// Vehicle Dynamics Engine - Track Geometry

#include "core/math/math_base.h"
#include "core/math/vec3.h"

//-------------------------
// Types
//-------------------------

typedef struct TrackGeometry TrackGeometry;

// Track segment (e.g., straight, curve, chicane)
typedef struct TrackSegment {
    vde_vec3 start_position;
    vde_vec3 end_position;
    vde_real length;             // Segment length (m)
    vde_real radius;             // Curve radius (m), 0 for straight
    vde_real banking;            // Banking angle (rad)
} TrackSegment;

//-------------------------
// API Functions
//-------------------------

VDE_API TrackGeometry* track_geometry_create(void);
VDE_API void track_geometry_destroy(TrackGeometry* geometry);

// Add track segments
VDE_API void track_geometry_add_segment(TrackGeometry* geometry, const TrackSegment* segment);
VDE_API int track_geometry_get_segment_count(const TrackGeometry* geometry);

// Query track position
VDE_API void track_geometry_get_closest_point(
    const TrackGeometry* geometry,
    const vde_vec3* position,
    vde_vec3* out_closest_point,
    vde_real* out_distance
);

// Get track centerline tangent at position
VDE_API void track_geometry_get_tangent(
    const TrackGeometry* geometry,
    const vde_vec3* position,
    vde_vec3* out_tangent
);
