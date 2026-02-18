#include "vehicle/brakes.h"
#include <stdlib.h>

//-------------------------
// Internal State
//-------------------------

struct Brakes {
    vde_real pressure;      // Brake input [0, 1]
    vde_real max_torque;    // Maximum brake torque per wheel (N*m)
    vde_real brake_balance; // Front brake bias [0, 1] (0.5 = 50/50 split)
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create brake system
 * 
 * Guiggiani Reference: Chapter 4
 *   Optimal brake balance depends on CG height, wheelbase, and grip
 * 
 * Output:
 *   - Returns pointer to Brakes, or NULL on failure
 */
Brakes* brakes_create(void) {
    Brakes* brakes = (Brakes*)malloc(sizeof(Brakes));
    if (!brakes) return NULL;
    
    brakes->pressure = (vde_real)0.0;
    brakes->max_torque = (vde_real)3000.0; // 3000 N*m typical
    brakes->brake_balance = (vde_real)0.6; // 60% front bias typical
    
    return brakes;
}

/**
 * Destroy brake system
 * 
 * Input:
 *   - brakes: Brakes to destroy (can be NULL)
 */
void brakes_destroy(Brakes* brakes) {
    if (!brakes) return;
    free(brakes);
}

//-------------------------
// Input
//-------------------------

/**
 * Set brake pressure/input
 * 
 * Input:
 *   - brakes: Brake system (must be non-NULL)
 *   - pressure: Brake input [0, 1] where 0=no braking, 1=max braking
 */
void brakes_set_pressure(Brakes* brakes, vde_real pressure) {
    if (!brakes) return;
    
    // Clamp to valid range
    if (pressure < (vde_real)0.0) pressure = (vde_real)0.0;
    if (pressure > (vde_real)1.0) pressure = (vde_real)1.0;
    
    brakes->pressure = pressure;
}

/**
 * Get current brake pressure
 * 
 * Input:
 *   - brakes: Brake system (must be non-NULL)
 * 
 * Output:
 *   - Returns brake pressure [0, 1]
 */
vde_real brakes_get_pressure(const Brakes* brakes) {
    if (!brakes) return (vde_real)0.0;
    return brakes->pressure;
}

//-------------------------
// Properties
//-------------------------

/**
 * Set maximum brake torque per wheel
 * 
 * Input:
 *   - brakes: Brake system (must be non-NULL)
 *   - max_torque: Maximum torque in N*m (typically 2000-5000)
 */
void brakes_set_max_torque(Brakes* brakes, vde_real max_torque) {
    if (!brakes) return;
    brakes->max_torque = max_torque;
}

/**
 * Get maximum brake torque
 * 
 * Input:
 *   - brakes: Brake system (must be non-NULL)
 * 
 * Output:
 *   - Returns max torque in N*m
 */
vde_real brakes_get_max_torque(const Brakes* brakes) {
    if (!brakes) return (vde_real)0.0;
    return brakes->max_torque;
}

//-------------------------
// Torque Computation
//-------------------------

/**
 * Compute brake torque for a specific wheel
 * 
 * Applies brake balance (front/rear distribution) based on wheel index.
 * 
 * Guiggiani Reference: Section 4.6
 *   Optimal balance: φ = b/l + (μ*h/l)
 * 
 * Input:
 *   - brakes: Brake system (must be non-NULL)
 *   - wheel_index: Wheel index (0=FL, 1=FR, 2=RL, 3=RR)
 * 
 * Output:
 *   - Returns brake torque for this wheel in N*m
 * 
 * Functionality:
 *   1. Apply pressure scaling
 *   2. Apply brake balance (front gets more)
 *   3. Return final torque
 */
vde_real brakes_compute_torque(const Brakes* brakes, int wheel_index) {
    if (!brakes) return (vde_real)0.0;
    
    // TODO: Implement proper brake torque computation with balance
    // Front wheels (0, 1) get brake_balance fraction
    // Rear wheels (2, 3) get (1 - brake_balance) fraction
    
    vde_real base_torque = brakes->pressure * brakes->max_torque;
    
    // Simple front/rear split
    if (wheel_index < 2) {
        // Front wheels
        return base_torque * brakes->brake_balance;
    } else {
        // Rear wheels
        return base_torque * ((vde_real)1.0 - brakes->brake_balance);
    }
}
