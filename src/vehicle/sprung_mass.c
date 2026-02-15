#include "vehicle/sprung_mass.h"
#include <stdlib.h>
#include <string.h>

//-------------------------
// Internal State
//-------------------------

struct SprungMass {
    vde_frame frame;            // Position and orientation
    vde_vec3 linear_velocity;   // Linear velocity (body frame)
    vde_vec3 angular_velocity;  // Angular velocity (body frame)
    vde_real mass;              // Mass (kg)
    vde_mat3 inertia;           // Inertia tensor (body frame)
    vde_vec3 force_accum;       // Accumulated forces
    vde_vec3 moment_accum;      // Accumulated moments
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create sprung mass (chassis)
 * 
 * Guiggiani Reference: Section 3.10, Chapter 9
 * 
 * Output:
 *   - Returns pointer to SprungMass, or NULL on failure
 */
SprungMass* sprung_mass_create(void) {
    SprungMass* sm = (SprungMass*)malloc(sizeof(SprungMass));
    if (!sm) return NULL;
    
    sm->frame = vde_frame_identity();
    sm->linear_velocity = vde_vec3_zero();
    sm->angular_velocity = vde_vec3_zero();
    sm->mass = (vde_real)1000.0; // 1000 kg typical
    sm->inertia = vde_mat3_identity();
    sm->force_accum = vde_vec3_zero();
    sm->moment_accum = vde_vec3_zero();
    
    return sm;
}

/**
 * Destroy sprung mass
 * 
 * Input:
 *   - sm: Sprung mass to destroy (can be NULL)
 */
void sprung_mass_destroy(SprungMass* sm) {
    if (!sm) return;
    free(sm);
}

//-------------------------
// State Accessors
//-------------------------

/**
 * Get rigid body frame (position and orientation)
 * 
 * Input:
 *   - sm: Sprung mass (must be non-NULL)
 *   - out_frame: Output buffer (must be non-NULL)
 * 
 * Output:
 *   - out_frame: Filled with current frame
 */
void sprung_mass_get_frame(const SprungMass* sm, vde_frame* out_frame) {
    if (!sm || !out_frame) return;
    *out_frame = sm->frame;
}

/**
 * Set rigid body frame
 * 
 * Input:
 *   - sm: Sprung mass (must be non-NULL)
 *   - frame: New frame (must be non-NULL)
 */
void sprung_mass_set_frame(SprungMass* sm, const vde_frame* frame) {
    if (!sm || !frame) return;
    sm->frame = *frame;
}

/**
 * Get linear and angular velocities
 * 
 * Input:
 *   - sm: Sprung mass (must be non-NULL)
 *   - out_linear: Output for linear velocity (can be NULL)
 *   - out_angular: Output for angular velocity (can be NULL)
 * 
 * Output:
 *   - out_linear: Filled with linear velocity if non-NULL
 *   - out_angular: Filled with angular velocity if non-NULL
 */
void sprung_mass_get_velocity(const SprungMass* sm, vde_vec3* out_linear, vde_vec3* out_angular) {
    if (!sm) return;
    if (out_linear) *out_linear = sm->linear_velocity;
    if (out_angular) *out_angular = sm->angular_velocity;
}

/**
 * Set linear and angular velocities
 * 
 * Input:
 *   - sm: Sprung mass (must be non-NULL)
 *   - linear: Linear velocity (can be NULL to keep current)
 *   - angular: Angular velocity (can be NULL to keep current)
 */
void sprung_mass_set_velocity(SprungMass* sm, const vde_vec3* linear, const vde_vec3* angular) {
    if (!sm) return;
    if (linear) sm->linear_velocity = *linear;
    if (angular) sm->angular_velocity = *angular;
}

//-------------------------
// Properties
//-------------------------

/**
 * Set mass
 * 
 * Input:
 *   - sm: Sprung mass (must be non-NULL)
 *   - mass: Mass in kg
 */
void sprung_mass_set_mass(SprungMass* sm, vde_real mass) {
    if (!sm) return;
    sm->mass = mass;
}

/**
 * Get mass
 * 
 * Input:
 *   - sm: Sprung mass (must be non-NULL)
 * 
 * Output:
 *   - Returns mass in kg
 */
vde_real sprung_mass_get_mass(const SprungMass* sm) {
    if (!sm) return (vde_real)0.0;
    return sm->mass;
}

/**
 * Set inertia tensor
 * 
 * Input:
 *   - sm: Sprung mass (must be non-NULL)
 *   - inertia: Inertia tensor in body frame (must be non-NULL)
 */
void sprung_mass_set_inertia(SprungMass* sm, const vde_mat3* inertia) {
    if (!sm || !inertia) return;
    sm->inertia = *inertia;
}

/**
 * Get inertia tensor
 * 
 * Input:
 *   - sm: Sprung mass (must be non-NULL)
 *   - out_inertia: Output buffer (must be non-NULL)
 * 
 * Output:
 *   - out_inertia: Filled with inertia tensor
 */
void sprung_mass_get_inertia(const SprungMass* sm, vde_mat3* out_inertia) {
    if (!sm || !out_inertia) return;
    *out_inertia = sm->inertia;
}

//-------------------------
// Force Accumulation
//-------------------------

/**
 * Reset accumulated forces and moments
 * 
 * Call at the beginning of each timestep before accumulating forces.
 * 
 * Input:
 *   - sm: Sprung mass (must be non-NULL)
 */
void sprung_mass_reset_forces(SprungMass* sm) {
    if (!sm) return;
    sm->force_accum = vde_vec3_zero();
    sm->moment_accum = vde_vec3_zero();
}

/**
 * Apply a force at a point on the body
 * 
 * Input:
 *   - sm: Sprung mass (must be non-NULL)
 *   - force: Force vector in body frame (must be non-NULL)
 *   - point_body: Application point in body frame (must be non-NULL)
 * 
 * Functionality:
 *   1. Add force to total force accumulator
 *   2. Compute moment = point × force
 *   3. Add moment to moment accumulator
 */
void sprung_mass_apply_force(SprungMass* sm, const vde_vec3* force, const vde_vec3* point_body) {
    if (!sm || !force || !point_body) return;
    
    // Accumulate force
    vde_vec3_add(&sm->force_accum, &sm->force_accum, force);
    
    // Compute and accumulate moment: M = r × F
    vde_vec3 moment;
    vde_vec3_cross(&moment, point_body, force);
    vde_vec3_add(&sm->moment_accum, &sm->moment_accum, &moment);
}

/**
 * Apply a pure moment
 * 
 * Input:
 *   - sm: Sprung mass (must be non-NULL)
 *   - moment: Moment vector in body frame (must be non-NULL)
 */
void sprung_mass_apply_moment(SprungMass* sm, const vde_vec3* moment) {
    if (!sm || !moment) return;
    vde_vec3_add(&sm->moment_accum, &sm->moment_accum, moment);
}

/**
 * Get accumulated forces and moments
 * 
 * Input:
 *   - sm: Sprung mass (must be non-NULL)
 *   - out_force: Output for total force (can be NULL)
 *   - out_moment: Output for total moment (can be NULL)
 * 
 * Output:
 *   - out_force: Filled with accumulated force if non-NULL
 *   - out_moment: Filled with accumulated moment if non-NULL
 */
void sprung_mass_get_accumulated_forces(const SprungMass* sm, vde_vec3* out_force, vde_vec3* out_moment) {
    if (!sm) return;
    if (out_force) *out_force = sm->force_accum;
    if (out_moment) *out_moment = sm->moment_accum;
} component
