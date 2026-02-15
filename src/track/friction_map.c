#include "track/friction_map.h"
#include <stdlib.h>
#include <math.h>

//-------------------------
// Internal State
//-------------------------

struct FrictionMap {
    vde_real* data;     // 2D grid of friction values
    int width;          // Grid width
    int height;         // Grid height
    vde_real min_x;     // World space bounds
    vde_real max_x;
    vde_real min_y;
    vde_real max_y;
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create friction map
 * 
 * Input:
 *   - width: Grid width (must be > 0)
 *   - height: Grid height (must be > 0)
 * 
 * Output:
 *   - Returns pointer to FrictionMap, or NULL on failure
 */
FrictionMap* friction_map_create(int width, int height) {
    if (width <= 0 || height <= 0) return NULL;
    
    FrictionMap* map = (FrictionMap*)malloc(sizeof(FrictionMap));
    if (!map) return NULL;
    
    map->width = width;
    map->height = height;
    map->data = (vde_real*)calloc(width * height, sizeof(vde_real));
    
    if (!map->data) {
        free(map);
        return NULL;
    }
    
    // Default bounds: 100m x 100m
    map->min_x = (vde_real)-50.0;
    map->max_x = (vde_real)50.0;
    map->min_y = (vde_real)-50.0;
    map->max_y = (vde_real)50.0;
    
    // Initialize with default friction
    for (int i = 0; i < width * height; i++) {
        map->data[i] = (vde_real)1.0;
    }
    
    return map;
}

/**
 * Destroy friction map
 * 
 * Input:
 *   - map: Friction map to destroy (can be NULL)
 */
void friction_map_destroy(FrictionMap* map) {
    if (!map) return;
    
    if (map->data) {
        free(map->data);
    }
    free(map);
}

//-------------------------
// Data Access
//-------------------------

/**
 * Set friction value at grid position
 * 
 * Input:
 *   - map: Friction map (must be non-NULL)
 *   - x: Grid X coordinate [0, width-1]
 *   - y: Grid Y coordinate [0, height-1]
 *   - friction: Friction value to set
 */
void friction_map_set_value(FrictionMap* map, int x, int y, vde_real friction) {
    if (!map || x < 0 || x >= map->width || y < 0 || y >= map->height) return;
    
    map->data[y * map->width + x] = friction;
}

/**
 * Get friction value at grid position
 * 
 * Input:
 *   - map: Friction map (must be non-NULL)
 *   - x: Grid X coordinate [0, width-1]
 *   - y: Grid Y coordinate [0, height-1]
 * 
 * Output:
 *   - Returns friction value, or 1.0 if out of bounds
 */
vde_real friction_map_get_value(const FrictionMap* map, int x, int y) {
    if (!map || x < 0 || x >= map->width || y < 0 || y >= map->height) {
        return (vde_real)1.0;
    }
    
    return map->data[y * map->width + x];
}

//-------------------------
// World Space Sampling
//-------------------------

/**
 * Sample friction at world position with bilinear interpolation
 * 
 * Input:
 *   - map: Friction map (must be non-NULL)
 *   - world_position: Position in world space (must be non-NULL)
 * 
 * Output:
 *   - Returns interpolated friction value
 * 
 * Functionality:
 *   1. Convert world position to grid coordinates
 *   2. Find surrounding 4 grid points
 *   3. Bilinear interpolate between them
 *   4. Return interpolated friction
 */
vde_real friction_map_sample(
    const FrictionMap* map,
    const vde_vec3* world_position
) {
    if (!map || !world_position) return (vde_real)1.0;
    
    // Convert world position to normalized [0, 1] coordinates
    vde_real norm_x = (world_position->x - map->min_x) / (map->max_x - map->min_x);
    vde_real norm_y = (world_position->y - map->min_y) / (map->max_y - map->min_y);
    
    // Clamp to valid range
    if (norm_x < (vde_real)0.0) norm_x = (vde_real)0.0;
    if (norm_x > (vde_real)1.0) norm_x = (vde_real)1.0;
    if (norm_y < (vde_real)0.0) norm_y = (vde_real)0.0;
    if (norm_y > (vde_real)1.0) norm_y = (vde_real)1.0;
    
    // Convert to grid coordinates
    vde_real grid_x = norm_x * (vde_real)(map->width - 1);
    vde_real grid_y = norm_y * (vde_real)(map->height - 1);
    
    // Get integer grid indices
    int x0 = (int)floor(grid_x);
    int y0 = (int)floor(grid_y);
    int x1 = x0 + 1;
    int y1 = y0 + 1;
    
    // Clamp to grid bounds
    if (x1 >= map->width) x1 = map->width - 1;
    if (y1 >= map->height) y1 = map->height - 1;
    
    // Get fractional parts for interpolation
    vde_real fx = grid_x - (vde_real)x0;
    vde_real fy = grid_y - (vde_real)y0;
    
    // Bilinear interpolation
    vde_real f00 = friction_map_get_value(map, x0, y0);
    vde_real f10 = friction_map_get_value(map, x1, y0);
    vde_real f01 = friction_map_get_value(map, x0, y1);
    vde_real f11 = friction_map_get_value(map, x1, y1);
    
    vde_real f0 = f00 * ((vde_real)1.0 - fx) + f10 * fx;
    vde_real f1 = f01 * ((vde_real)1.0 - fx) + f11 * fx;
    vde_real result = f0 * ((vde_real)1.0 - fy) + f1 * fy;
    
    return result;
}

/**
 * Set map world space bounds
 * 
 * Input:
 *   - map: Friction map (must be non-NULL)
 *   - min_x, max_x: X bounds in world space
 *   - min_y, max_y: Y bounds in world space
 */
void friction_map_set_bounds(
    FrictionMap* map,
    vde_real min_x, vde_real max_x,
    vde_real min_y, vde_real max_y
) {
    if (!map) return;
    
    map->min_x = min_x;
    map->max_x = max_x;
    map->min_y = min_y;
    map->max_y = max_y;
}
