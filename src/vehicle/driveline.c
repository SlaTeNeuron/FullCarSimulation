#include "vehicle/driveline.h"
#include <stdlib.h>

//-------------------------
// Internal State
//-------------------------

struct Driveline {
    vde_real throttle;              // Throttle input [0, 1]
    vde_real engine_torque;         // Maximum engine torque (N*m)
    vde_real gear_ratios[7];        // [0]=1st .. [5]=6th, [6]=reverse
    int      current_gear_index;    // Active gear index into gear_ratios[]
    vde_real final_drive;           // Final drive ratio
    vde_real drivetrain_efficiency; // Transmission efficiency [0, 1]
    int      drive_config;          // 0=FWD, 1=RWD, 2=AWD
    int      driven_wheels[4];      // 1=driven, 0=not (FL, FR, RL, RR)
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
    
    driveline->throttle              = (vde_real)0.0;
    driveline->engine_torque         = (vde_real)72.0;  // N*m (restricted R6)
    // Yamaha R6 gear ratios: 1st–5th, 6th unused, reverse
    driveline->gear_ratios[0] = (vde_real)2.615;
    driveline->gear_ratios[1] = (vde_real)1.812;
    driveline->gear_ratios[2] = (vde_real)1.409;
    driveline->gear_ratios[3] = (vde_real)1.130;
    driveline->gear_ratios[4] = (vde_real)0.935;
    driveline->gear_ratios[5] = (vde_real)0.0;    // 6th gear unused
    driveline->gear_ratios[6] = (vde_real)-3.0;   // reverse
    driveline->current_gear_index    = 0;          // start in 1st
    driveline->final_drive           = (vde_real)3.77;
    driveline->drivetrain_efficiency = (vde_real)0.91;
    driveline->drive_config          = 1; // RWD
    // RWD: rear wheels driven
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
    driveline->throttle = vde_clamp(throttle, (vde_real)0.0, (vde_real)1.0);
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

/** Set maximum engine torque (N*m). */
void driveline_set_engine_torque(Driveline* driveline, vde_real torque) {
    if (!driveline) return;
    driveline->engine_torque = torque;
}

/** Set a single active gear ratio (convenience — sets current_gear_index slot to ratio). */
void driveline_set_gear_ratio(Driveline* driveline, vde_real ratio) {
    if (!driveline) return;
    driveline->gear_ratios[driveline->current_gear_index] = ratio;
}

/**
 * Load all gear ratios from an array.
 * ratios[0] = 1st gear, ..., ratios[5] = 6th gear, ratios[6] = reverse.
 * Only min(count, 7) entries are written.
 */
void driveline_set_gear_ratios(Driveline* driveline, const vde_real* ratios, int count) {
    if (!driveline || !ratios) return;
    int n = (count < 7) ? count : 7;
    for (int i = 0; i < n; i++) {
        driveline->gear_ratios[i] = ratios[i];
    }
}

/** Set final drive ratio. */
void driveline_set_final_drive(Driveline* driveline, vde_real ratio) {
    if (!driveline) return;
    driveline->final_drive = ratio;
}

/**
 * Set drive configuration and update driven-wheels mask.
 * config: 0=FWD, 1=RWD, 2=AWD
 */
void driveline_set_drive_config(Driveline* driveline, int config) {
    if (!driveline) return;
    driveline->drive_config = config;
    switch (config) {
    case 0: // FWD
        driveline->driven_wheels[0] = 1;
        driveline->driven_wheels[1] = 1;
        driveline->driven_wheels[2] = 0;
        driveline->driven_wheels[3] = 0;
        break;
    case 2: // AWD
        driveline->driven_wheels[0] = 1;
        driveline->driven_wheels[1] = 1;
        driveline->driven_wheels[2] = 1;
        driveline->driven_wheels[3] = 1;
        break;
    default: // RWD (1)
        driveline->driven_wheels[0] = 0;
        driveline->driven_wheels[1] = 0;
        driveline->driven_wheels[2] = 1;
        driveline->driven_wheels[3] = 1;
        break;
    }
}

int driveline_get_drive_config(const Driveline* driveline) {
    if (!driveline) return 1; // RWD default
    return driveline->drive_config;
}

/** Set drivetrain efficiency factor [0, 1]. */
void driveline_set_drivetrain_efficiency(Driveline* driveline, vde_real efficiency) {
    if (!driveline) return;
    driveline->drivetrain_efficiency = vde_clamp(efficiency, (vde_real)0.0, (vde_real)1.0);
}

vde_real driveline_get_drivetrain_efficiency(const Driveline* driveline) {
    if (!driveline) return (vde_real)1.0;
    return driveline->drivetrain_efficiency;
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
    if (!driveline->driven_wheels[wheel_index]) return (vde_real)0.0;

    // Count driven wheels for torque splitting
    int num_driven = 0;
    for (int i = 0; i < 4; i++) num_driven += driveline->driven_wheels[i];
    if (num_driven == 0) return (vde_real)0.0;

    // Active gear ratio from array
    vde_real active_ratio = driveline->gear_ratios[driveline->current_gear_index];

    // Total torque at wheel hubs (after gearbox, final drive, efficiency)
    // Guiggiani Sec. 3.11.4: T_wheel = T_engine × i_gear × i_final × η
    vde_real total_torque = driveline->throttle
                          * driveline->engine_torque
                          * active_ratio
                          * driveline->final_drive
                          * driveline->drivetrain_efficiency;

    // Open differential: equal split among driven wheels
    return total_torque / (vde_real)num_driven;
}
