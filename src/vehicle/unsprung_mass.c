#include "vehicle/unsprung_mass.h"
#include <stdlib.h>

//-------------------------
// Internal State
//-------------------------

struct UnsprungMass {
    vde_vec3 position;       // Position (world frame)
    vde_vec3 velocity;       // Velocity (world frame)
    vde_real wheel_spin;     // Wheel angular velocity (rad/s)
    vde_real mass;           // Mass (kg)
    vde_vec3 force_accum;    // Accumulated forces
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create unsprung mass (wheel assembly)
 * 
 * Guiggiani Reference: Section 3.10
 * 
 * Output:
 *   - Returns pointer to UnsprungMass, or NULL on failure
 */
UnsprungMass* unsprung_mass_create(void) {
    UnsprungMass* um = (UnsprungMass*)malloc(sizeof(UnsprungMass));
    if (!um) return NULL;
    
    um->position = vde_vec3_zero();
    um->velocity = vde_vec3_zero();
    um->wheel_spin = (vde_real)0.0;
    um->mass = (vde_real)25.0; // 25 kg typical for wheel+tire+hub
    um->force_accum = vde_vec3_zero();
    
    return um;
}

/**
 * Destroy unsprung mass
 * 
 * Input:
 *   - um: Unsprung mass to destroy (can be NULL)
 */
void unsprung_mass_destroy(UnsprungMass* um) {
    if (!um) return;
    free(um);
}

//-------------------------
// State Accessors
//-------------------------

/**
 * Get position
 * 
 * Input:
 *   - um: Unsprung mass (must be non-NULL)
 *   - out_position: Output buffer (must be non-NULL)
 * 
 * Output:
 *   - out_position: Filled with position
 */
void unsprung_mass_get_position(const UnsprungMass* um, vde_vec3* out_position) {
    if (!um || !out_position) return;
    *out_position = um->position;
}

/**
 * Set position
 * 
 * Input:
 *   - um: Unsprung mass (must be non-NULL)
 *   - position: New position (must be non-NULL)
 */
void unsprung_mass_set_position(UnsprungMass* um, const vde_vec3* position) {
    if (!um || !position) return;
    um->position = *position;
}

/**
 * Get velocity
 * 
 * Input:
 *   - um: Unsprung mass (must be non-NULL)
 *   - out_velocity: Output buffer (must be non-NULL)
 * 
 * Output:
 *   - out_velocity: Filled with velocity
 */
void unsprung_mass_get_velocity(const UnsprungMass* um, vde_vec3* out_velocity) {
    if (!um || !out_velocity) return;
    *out_velocity = um->velocity;
}

/**
 * Set velocity
 * 
 * Input:
 *   - um: Unsprung mass (must be non-NULL)
 *   - velocity: New velocity (must be non-NULL)
 */
void unsprung_mass_set_velocity(UnsprungMass* um, const vde_vec3* velocity) {
    if (!um || !velocity) return;
    um->velocity = *velocity;
}

/**
 * Get wheel spin rate
 * 
 * Input:
 *   - um: Unsprung mass (must be non-NULL)
 * 
 * Output:
 *   - Returns wheel angular velocity in rad/s
 */
vde_real unsprung_mass_get_wheel_spin_rate(const UnsprungMass* um) {
    if (!um) return (vde_real)0.0;
    return um->wheel_spin;
}

/**
 * Set wheel spin rate
 * 
 * Input:
 *   - um: Unsprung mass (must be non-NULL)
 *   - spin_rate: Wheel angular velocity in rad/s
 */
void unsprung_mass_set_wheel_spin_rate(UnsprungMass* um, vde_real spin_rate) {
    if (!um) return;
    um->wheel_spin = spin_rate;
}

//-------------------------
// Properties
//-------------------------

/**
 * Set mass
 * 
 * Input:
 *   - um: Unsprung mass (must be non-NULL)
 *   - mass: Mass in kg (typically 20-30 kg)
 */
void unsprung_mass_set_mass(UnsprungMass* um, vde_real mass) {
    if (!um) return;
    um->mass = mass;
}

/**
 * Get mass
 * 
 * Input:
 *   - um: Unsprung mass (must be non-NULL)
 * 
 * Output:
 *   - Returns mass in kg
 */
vde_real unsprung_mass_get_mass(const UnsprungMass* um) {
    if (!um) return (vde_real)0.0;
    return um->mass;
}

//-------------------------
// Force Accumulation
//-------------------------

/**
 * Reset accumulated forces
 * 
 * Input:
 *   - um: Unsprung mass (must be non-NULL)
 */
void unsprung_mass_reset_forces(UnsprungMass* um) {
    if (!um) return;
    um->force_accum = vde_vec3_zero();
}

/**
 * Apply a force
 * 
 * Input:
 *   - um: Unsprung mass (must be non-NULL)
 *   - force: Force vector (must be non-NULL)
 */
void unsprung_mass_apply_force(UnsprungMass* um, const vde_vec3* force) {
    if (!um || !force) return;
    vde_vec3_add(&um->force_accum, &um->force_accum, force);
}

/**
 * Get accumulated force
 * 
 * Input:
 *   - um: Unsprung mass (must be non-NULL)
 *   - out_force: Output buffer (must be non-NULL)
 * 
 * Output:
 *   - out_force: Filled with accumulated force
 */
void unsprung_mass_get_accumulated_force(const UnsprungMass* um, vde_vec3* out_force) {
    if (!um || !out_force) return;
    *out_force = um->force_accum;
}
