#pragma once
// Vehicle Dynamics Engine - Control Input System

#include "core/math/math_base.h"

//-------------------------
// Types
//-------------------------

typedef struct ControlInput ControlInput;

// Control input state
typedef struct ControlState {
    vde_real throttle;           // Throttle input [0, 1]
    vde_real brake;              // Brake input [0, 1]
    vde_real steering;           // Steering input [-1, 1] (left/right)
    vde_real clutch;             // Clutch input [0, 1]
    int gear;                    // Current gear (-1=reverse, 0=neutral, 1+=forward)
} ControlState;

//-------------------------
// API Functions
//-------------------------

VDE_API ControlInput* control_input_create(void);
VDE_API void control_input_destroy(ControlInput* control);

// Set individual inputs
VDE_API void control_input_set_throttle(ControlInput* control, vde_real throttle);
VDE_API void control_input_set_brake(ControlInput* control, vde_real brake);
VDE_API void control_input_set_steering(ControlInput* control, vde_real steering);
VDE_API void control_input_set_clutch(ControlInput* control, vde_real clutch);
VDE_API void control_input_set_gear(ControlInput* control, int gear);

// Set all inputs at once
VDE_API void control_input_set_state(ControlInput* control, const ControlState* state);

// Get current state
VDE_API void control_input_get_state(const ControlInput* control, ControlState* out_state);

// Input filtering/smoothing
VDE_API void control_input_set_filter_time(ControlInput* control, vde_real time_constant);
VDE_API void control_input_update(ControlInput* control, vde_real dt);