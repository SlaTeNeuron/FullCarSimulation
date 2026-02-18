#include "input/control.h"
#include <stdlib.h>
#include <string.h>

//-------------------------
// Internal State
//-------------------------

struct ControlInput {
    ControlState current;        // Current control state
    ControlState target;         // Target (unfiltered) state
    vde_real filter_time;        // Filter time constant
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create control input system
 * 
 * Output:
 *   - Returns pointer to ControlInput, or NULL on failure
 */
ControlInput* control_input_create(void) {
    ControlInput* control = (ControlInput*)malloc(sizeof(ControlInput));
    if (!control) return NULL;
    
    // Initialize to neutral
    memset(&control->current, 0, sizeof(ControlState));
    memset(&control->target, 0, sizeof(ControlState));
    control->filter_time = (vde_real)0.05; // 50ms default filter
    
    return control;
}

/**
 * Destroy control input system
 * 
 * Input:
 *   - control: Control input to destroy (can be NULL)
 */
void control_input_destroy(ControlInput* control) {
    if (!control) return;
    free(control);
}

//-------------------------
// Individual Setters
//-------------------------

/**
 * Set throttle input
 * 
 * Input:
 *   - control: Control input (must be non-NULL)
 *   - throttle: Throttle value [0, 1]
 */
void control_input_set_throttle(ControlInput* control, vde_real throttle) {
    if (!control) return;
    
    // Clamp to valid range
    if (throttle < (vde_real)0.0) throttle = (vde_real)0.0;
    if (throttle > (vde_real)1.0) throttle = (vde_real)1.0;
    
    control->target.throttle = throttle;
}

/**
 * Set brake input
 * 
 * Input:
 *   - control: Control input (must be non-NULL)
 *   - brake: Brake value [0, 1]
 */
void control_input_set_brake(ControlInput* control, vde_real brake) {
    if (!control) return;
    
    // Clamp to valid range
    if (brake < (vde_real)0.0) brake = (vde_real)0.0;
    if (brake > (vde_real)1.0) brake = (vde_real)1.0;
    
    control->target.brake = brake;
}

/**
 * Set steering input
 * 
 * Input:
 *   - control: Control input (must be non-NULL)
 *   - steering: Steering value [-1, 1] (left negative, right positive)
 */
void control_input_set_steering(ControlInput* control, vde_real steering) {
    if (!control) return;
    
    // Clamp to valid range
    if (steering < (vde_real)-1.0) steering = (vde_real)-1.0;
    if (steering > (vde_real)1.0) steering = (vde_real)1.0;
    
    control->target.steering = steering;
}

/**
 * Set clutch input
 * 
 * Input:
 *   - control: Control input (must be non-NULL)
 *   - clutch: Clutch value [0, 1] (0=engaged, 1=disengaged)
 */
void control_input_set_clutch(ControlInput* control, vde_real clutch) {
    if (!control) return;
    
    // Clamp to valid range
    if (clutch < (vde_real)0.0) clutch = (vde_real)0.0;
    if (clutch > (vde_real)1.0) clutch = (vde_real)1.0;
    
    control->target.clutch = clutch;
}

/**
 * Set gear
 * 
 * Input:
 *   - control: Control input (must be non-NULL)
 *   - gear: Gear number (-1=reverse, 0=neutral, 1+=forward)
 */
void control_input_set_gear(ControlInput* control, int gear) {
    if (!control) return;
    control->target.gear = gear;
}

//-------------------------
// Bulk State Operations
//-------------------------

/**
 * Set all control inputs at once
 * 
 * Input:
 *   - control: Control input (must be non-NULL)
 *   - state: Control state to set (must be non-NULL)
 */
void control_input_set_state(ControlInput* control, const ControlState* state) {
    if (!control || !state) return;
    control->target = *state;
}

/**
 * Get current control state
 * 
 * Input:
 *   - control: Control input (must be non-NULL)
 *   - out_state: Output buffer (must be non-NULL)
 * 
 * Output:
 *   - out_state: Filled with current (filtered) control state
 */
void control_input_get_state(const ControlInput* control, ControlState* out_state) {
    if (!control || !out_state) return;
    *out_state = control->current;
}

//-------------------------
// Filtering
//-------------------------

/**
 * Set input filter time constant
 * 
 * Input:
 *   - control: Control input (must be non-NULL)
 *   - time_constant: Filter time constant in seconds (0 = no filtering)
 */
void control_input_set_filter_time(ControlInput* control, vde_real time_constant) {
    if (!control) return;
    control->filter_time = time_constant;
}

/**
 * Update control inputs with filtering
 * 
 * Applies first-order low-pass filter to smooth inputs.
 * 
 * Input:
 *   - control: Control input (must be non-NULL)
 *   - dt: Time step (must be > 0)
 * 
 * Functionality:
 *   For each input:
 *     current = current + (target - current) * (dt / filter_time)
 *   Gear changes are not filtered (instantaneous)
 */
void control_input_update(ControlInput* control, vde_real dt) {
    if (!control || dt <= (vde_real)0.0) return;
    
    if (control->filter_time <= (vde_real)0.0) {
        // No filtering
        control->current = control->target;
        return;
    }
    
    // TODO: Implement first-order filter
    // alpha = dt / (dt + filter_time)
    // current = current * (1 - alpha) + target * alpha
    
    // For now, no filtering
    control->current = control->target;
}
