#include "vehicle/driveline.h"
#include <stdlib.h>

//-------------------------
// Internal State
//-------------------------

struct Driveline {
    vde_real throttle;         // Throttle input [0, 1]
    vde_real engine_torque;    // Maximum engine torque (N*m)
    vde_real gear_ratio;       // Current gear ratio
    vde_real final_drive;      // Final drive ratio
    int driven_wheels[4];      // Which wheels are driven (1=driven, 0=not)
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create driveline system
 * 
 * Guiggiani Reference: Section 3.11.4, 6.1, 7.1
 * 
 * Output:
 *   - Returns pointer to Driveline, or NULL on failure
 */
Driveline* driveline_create(void) {
    Driveline* driveline = (Driveline*)malloc(sizeof(Driveline));
    if (!driveline) return NULL;
    
    driveline->throttle = (vde_real)0.0;
    driveline->engine_torque = (vde_real)300.0; // 300 N*m typical
    driveline->gear_ratio = (vde_real)3.0;      // 3:1 typical for 2nd gear
    driveline->final_drive = (vde_real)3.5;     // 3.5:1 typical
    
    // RWD by default (rear wheels driven)
    driveline->driven_wheels[0] = 0; // FL
    driveline->driven_wheels[1] = 0; // FR
    driveline->driven_wheels[2] = 1; // RL
    driveline->driven_wheels[3] = 1; // RR
    
    return driveline;
}

/**
 * Destroy driveline system
 * 
 * Input:
 *   - driveline: Driveline to destroy (can be NULL)
 */
void driveline_destroy(Driveline* driveline) {
    if (!driveline) return;
    free(driveline);
}

//-------------------------
// Input
//-------------------------

/**
 * Set throttle input
 * 
 * Input:
 *   - driveline: Driveline system (must be non-NULL)
 *   - throttle: Throttle input [0, 1] where 0=idle, 1=full throttle
 */
void driveline_set_throttle(Driveline* driveline, vde_real throttle) {
    if (!driveline) return;
    
    // Clamp to valid range
    if (throttle < (vde_real)0.0) throttle = (vde_real)0.0;
    if (throttle > (vde_real)1.0) throttle = (vde_real)1.0;
    
    driveline->throttle = throttle;
}

/**
 * Get current throttle
 * 
 * Input:
 *   - driveline: Driveline system (must be non-NULL)
 * 
 * Output:
 *   - Returns throttle [0, 1]
 */
vde_real driveline_get_throttle(const Driveline* driveline) {
    if (!driveline) return (vde_real)0.0;
    return driveline->throttle;
}

//-------------------------
// Properties
//-------------------------

/**
 * Set maximum engine torque
 * 
 * Input:
 *   - driveline: Driveline system (must be non-NULL)
 *   - torque: Max engine torque in N*m (typically 200-500 for road cars)
 */
void driveline_set_engine_torque(Driveline* driveline, vde_real torque) {
    if (!driveline) return;
    driveline->engine_torque = torque;
}

/**
 * Set gear ratio
 * 
 * Input:
 *   - driveline: Driveline system (must be non-NULL)
 *   - ratio: Gear ratio (e.g., 3.0 for 3:1)
 */
void driveline_set_gear_ratio(Driveline* driveline, vde_real ratio) {
    if (!driveline) return;
    driveline->gear_ratio = ratio;
}

/**
 * Set final drive ratio
 * 
 * Input:
 *   - driveline: Driveline system (must be non-NULL)
 *   - ratio: Final drive ratio (e.g., 3.5 for 3.5:1)
 */
void driveline_set_final_drive(Driveline* driveline, vde_real ratio) {
    if (!driveline) return;
    driveline->final_drive = ratio;
}

//-------------------------
// Torque Computation
//-------------------------

/**
 * Compute drive torque for a specific wheel
 * 
 * Applies gear ratios and differential splitting.
 * 
 * Guiggiani Reference: Section 3.11.4 (differential mechanism)
 * 
 * Input:
 *   - driveline: Driveline system (must be non-NULL)
 *   - wheel_index: Wheel index (0=FL, 1=FR, 2=RL, 3=RR)
 * 
 * Output:
 *   - Returns drive torque for this wheel in N*m
 * 
 * Functionality:
 *   1. Check if wheel is driven
 *   2. Apply throttle scaling
 *   3. Apply gear ratio and final drive
 *   4. Split torque among driven wheels (differential effect)
 */
vde_real driveline_compute_wheel_torque(const Driveline* driveline, int wheel_index) {
    if (!driveline || wheel_index < 0 || wheel_index >= 4) {
        return (vde_real)0.0;
    }
    
    // Check if this wheel is driven
    if (!driveline->driven_wheels[wheel_index]) {
        return (vde_real)0.0;
    }
    
    // TODO: Implement proper differential torque splitting
    // For open diff: equal torque to both wheels
    // For LSD: torque bias based on speed difference
    
    // Count driven wheels
    int num_driven = 0;
    for (int i = 0; i < 4; i++) {
        num_driven += driveline->driven_wheels[i];
    }
    
    if (num_driven == 0) return (vde_real)0.0;
    
    // Total torque at wheels
    vde_real total_torque = driveline->throttle * driveline->engine_torque 
                           * driveline->gear_ratio * driveline->final_drive;
    
    // Split evenly among driven wheels (open differential)
    return total_torque / (vde_real)num_driven;
} (engine, transmission, differentials)
