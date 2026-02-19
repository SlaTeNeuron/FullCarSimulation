#include "vehicle/brakes.h"
#include <stdlib.h>

//-------------------------
// Internal State
//-------------------------

struct Brakes {
    vde_real pressure;          // Brake input [0, 1]
    vde_real max_torque_front;  // Maximum brake torque per front wheel (N*m)
    vde_real max_torque_rear;   // Maximum brake torque per rear  wheel (N*m)
    vde_real brake_balance;     // Front fraction [0, 1] (Guiggiani Sec. 4.6)
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
    
    brakes->pressure         = (vde_real)0.0;
    brakes->max_torque_front = (vde_real)1200.0; // N*m per front wheel (TBRe defaults)
    brakes->max_torque_rear  = (vde_real) 800.0; // N*m per rear  wheel
    brakes->brake_balance    = (vde_real)0.65;   // 65% front (Guiggiani Sec. 4.6)
    
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
    brakes->pressure = vde_clamp(pressure, (vde_real)0.0, (vde_real)1.0);
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

/** Set maximum brake torque for front wheels (N*m each). */
void brakes_set_max_torque_front(Brakes* brakes, vde_real max_torque) {
    if (!brakes) return;
    brakes->max_torque_front = max_torque;
}

vde_real brakes_get_max_torque_front(const Brakes* brakes) {
    if (!brakes) return (vde_real)0.0;
    return brakes->max_torque_front;
}

/** Set maximum brake torque for rear wheels (N*m each). */
void brakes_set_max_torque_rear(Brakes* brakes, vde_real max_torque) {
    if (!brakes) return;
    brakes->max_torque_rear = max_torque;
}

vde_real brakes_get_max_torque_rear(const Brakes* brakes) {
    if (!brakes) return (vde_real)0.0;
    return brakes->max_torque_rear;
}

/** Convenience: set the same torque limit for all four wheels. */
void brakes_set_max_torque(Brakes* brakes, vde_real max_torque) {
    if (!brakes) return;
    brakes->max_torque_front = max_torque;
    brakes->max_torque_rear  = max_torque;
}

vde_real brakes_get_max_torque(const Brakes* brakes) {
    // Returns the higher of front/rear (representative maximum)
    if (!brakes) return (vde_real)0.0;
    return vde_max(brakes->max_torque_front, brakes->max_torque_rear);
}

/** Front/rear brake balance: fraction on front axle [0, 1]. */
void brakes_set_brake_balance(Brakes* brakes, vde_real balance) {
    if (!brakes) return;
    brakes->brake_balance = vde_clamp(balance, (vde_real)0.0, (vde_real)1.0);
}

vde_real brakes_get_brake_balance(const Brakes* brakes) {
    if (!brakes) return (vde_real)0.5;
    return brakes->brake_balance;
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
    
    // Front wheels (0=FL, 1=FR) use max_torque_front directly.
    // Rear  wheels (2=RL, 3=RR) use max_torque_rear  directly.
    // Guiggiani Sec. 4.6: optimal balance φ = c/L + μ·h/L
    if (wheel_index < 2) {
        return brakes->pressure * brakes->max_torque_front;
    } else {
        return brakes->pressure * brakes->max_torque_rear;
    }
}
