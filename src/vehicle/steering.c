#include "vehicle/steering.h"
#include <stdlib.h>

//-------------------------
// Internal State
//-------------------------

struct Steering {
    vde_real angle;             // Steering wheel angle (rad)
    vde_real ratio;             // Steering ratio (steering wheel / road wheel)
    vde_real max_angle;         // Max road-wheel steer angle (rad)
    vde_real ackermann_factor;  // 0 = parallel, 1 = full Ackermann
    vde_real wheelbase;         // Wheelbase (m) — for Ackermann geometry
    vde_real track_width;       // Track width (m) — for Ackermann geometry
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
    
    steering->angle            = (vde_real)0.0;
    steering->ratio            = (vde_real)3.5;   // 3.5:1 direct FSAE rack
    steering->max_angle        = (vde_real)0.349; // ~20 deg road-wheel lock
    steering->ackermann_factor = (vde_real)0.75;  // partial Ackermann
    steering->wheelbase        = (vde_real)1.530;
    steering->track_width      = (vde_real)1.195; // avg of front/rear
    
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

/** Set max road-wheel steer angle (rad). */
void steering_set_max_angle(Steering* steering, vde_real max_rad) {
    if (!steering) return;
    steering->max_angle = max_rad;
}

vde_real steering_get_max_angle(const Steering* steering) {
    if (!steering) return (vde_real)0.349;
    return steering->max_angle;
}

/** Ackermann factor: 0 = parallel steer, 1 = full Ackermann geometry. */
void steering_set_ackermann_factor(Steering* steering, vde_real factor) {
    if (!steering) return;
    steering->ackermann_factor = vde_clamp(factor, (vde_real)0.0, (vde_real)1.0);
}

vde_real steering_get_ackermann_factor(const Steering* steering) {
    if (!steering) return (vde_real)0.0;
    return steering->ackermann_factor;
}

/** Set wheelbase used in Ackermann geometry (m). */
void steering_set_wheelbase(Steering* steering, vde_real wheelbase) {
    if (!steering) return;
    steering->wheelbase = wheelbase;
}

/** Set track width used in Ackermann geometry (m). */
void steering_set_track_width(Steering* steering, vde_real track_width) {
    if (!steering) return;
    steering->track_width = track_width;
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

    if (corner_index >= 2) return (vde_real)0.0;  // rear wheels: no steer

    // Convert steering-wheel angle → base road-wheel angle, clamped to lock
    vde_real base = vde_clamp(steering->angle / steering->ratio,
                               -steering->max_angle, steering->max_angle);

    if (vde_abs(base) < VDE_EPS) return (vde_real)0.0;

    // Ackermann correction (Guiggiani Sec. 3.2.3)
    // Ideal Ackermann: outer wheel angle < inner wheel angle
    // cot(delta_outer) = L/R + t/(2R), cot(delta_inner) = L/R - t/(2R)
    // Linearised for small angles: delta_ackermann ≈ t/(2L) * delta²
    // We blend between parallel (factor=0) and full Ackermann (factor=1)
    vde_real L = steering->wheelbase;
    vde_real t = steering->track_width;
    vde_real f = steering->ackermann_factor;

    // Turn radius from base angle: R = L / tan(|base|) ≈ L / |base|
    // Left wheel is index 0, right wheel is index 1.
    // Positive base = steer right → right wheel is inner.
    vde_real sign = (base > (vde_real)0.0) ? (vde_real)1.0 : (vde_real)-1.0;
    int is_inner = ((base > (vde_real)0.0) && (corner_index == 1)) ||
                   ((base < (vde_real)0.0) && (corner_index == 0));

    if (f < VDE_EPS) return base;  // parallel steer, no correction

    // Full Ackermann angle for this wheel:
    // delta_inner  = atan(L / (L/tan(|base|) - t/2))
    // delta_outer  = atan(L / (L/tan(|base|) + t/2))
    vde_real abs_base = vde_abs(base);
    vde_real R_centre = (abs_base > VDE_EPS) ? (L / vde_tan(abs_base)) : (vde_real)1e6;
    vde_real half_t   = t * (vde_real)0.5;
    vde_real R_inner  = R_centre - half_t;
    vde_real R_outer  = R_centre + half_t;

    vde_real ack_inner = (R_inner > VDE_EPS) ? vde_atan(L / R_inner) : abs_base;
    vde_real ack_outer = (R_outer > VDE_EPS) ? vde_atan(L / R_outer) : abs_base;

    vde_real ack_angle = is_inner ? ack_inner : ack_outer;
    // Blend between parallel (base) and full Ackermann
    vde_real result = vde_lerp(abs_base, ack_angle, f);
    return sign * result;
}
