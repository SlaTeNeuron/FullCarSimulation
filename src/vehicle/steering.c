#include "vehicle/steering.h"
#include <stdlib.h>
#include <math.h>

//-------------------------
// Internal State
//-------------------------

struct Steering {
    vde_real angle;         // Steering wheel angle (rad)
    vde_real ratio;         // Steering ratio (wheel turns / road wheel turns)
    vde_real wheelbase;     // Wheelbase for Ackermann geometry (m)
    vde_real track_width;   // Track width (m)
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create steering system
 * 
 * Guiggiani Reference: Section 3.2.3, 6.18
 *   Ackermann geometry ensures kinematically correct turning
 * 
 * Output:
 *   - Returns pointer to Steering, or NULL on failure
 */
Steering* steering_create(void) {
    Steering* steering = (Steering*)malloc(sizeof(Steering));
    if (!steering) return NULL;
    
    steering->angle = (vde_real)0.0;
    steering->ratio = (vde_real)15.0;   // 15:1 typical
    steering->wheelbase = (vde_real)2.5; // 2.5m typical
    steering->track_width = (vde_real)1.5; // 1.5m typical
    
    return steering;
}

/**
 * Destroy steering system
 * 
 * Input:
 *   - steering: Steering to destroy (can be NULL)
 */
void steering_destroy(Steering* steering) {
    if (!steering) return;
    free(steering);
}

//-------------------------
// Input
//-------------------------

/**
 * Set steering angle
 * 
 * Input:
 *   - steering: Steering system (must be non-NULL)
 *   - angle_rad: Steering wheel angle in radians
 */
void steering_set_angle(Steering* steering, vde_real angle_rad) {
    if (!steering) return;
    steering->angle = angle_rad;
}

/**
 * Get current steering angle
 * 
 * Input:
 *   - steering: Steering system (must be non-NULL)
 * 
 * Output:
 *   - Returns steering wheel angle in radians
 */
vde_real steering_get_angle(const Steering* steering) {
    if (!steering) return (vde_real)0.0;
    return steering->angle;
}

//-------------------------
// Properties
//-------------------------

/**
 * Set steering ratio
 * 
 * Input:
 *   - steering: Steering system (must be non-NULL)
 *   - ratio: Steering ratio (typically 12-20 for road cars)
 */
void steering_set_ratio(Steering* steering, vde_real ratio) {
    if (!steering) return;
    steering->ratio = ratio;
}

/**
 * Get steering ratio
 * 
 * Input:
 *   - steering: Steering system (must be non-NULL)
 * 
 * Output:
 *   - Returns steering ratio
 */
vde_real steering_get_ratio(const Steering* steering) {
    if (!steering) return (vde_real)1.0;
    return steering->ratio;
}

//-------------------------
// Wheel Angle Computation
//-------------------------

/**
 * Compute wheel steer angle for a specific corner
 * 
 * Applies Ackermann geometry for kinematically correct steering.
 * 
 * Guiggiani Reference: Section 3.2.3
 *   Inner wheel: cot(δi) = l/R + t/(2R)
 *   Outer wheel: cot(δo) = l/R - t/(2R)
 * 
 * Input:
 *   - steering: Steering system (must be non-NULL)
 *   - corner_index: Corner index (0=FL, 1=FR, 2=RL, 3=RR)
 * 
 * Output:
 *   - Returns wheel steer angle in radians
 * 
 * Functionality:
 *   1. Convert steering wheel angle to road wheel angle
 *   2. Apply Ackermann correction based on corner
 *   3. Return final wheel angle
 */
vde_real steering_compute_wheel_angle(const Steering* steering, int corner_index) {
    if (!steering) return (vde_real)0.0;
    
    // Convert steering wheel angle to road wheel angle
    vde_real base_angle = steering->angle / steering->ratio;
    
    // TODO: Implement proper Ackermann geometry
    // For now, simplified: inner wheel gets more angle, outer less
    // Front wheels (0, 1) get steering, rear wheels (2, 3) get zero
    
    if (corner_index >= 2) {
        // Rear wheels: no steering
        return (vde_real)0.0;
    }
    
    // Simple Ackermann approximation
    // Left wheel (0) is inner when steering right (positive angle)
    // Right wheel (1) is inner when steering left (negative angle)
    
    return base_angle; // Simplified - needs proper Ackermann
}
