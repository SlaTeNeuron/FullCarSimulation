#include "vehicle/vehicle.h"
#include <stdlib.h>
#include <math.h>

//-------------------------
// Internal State
//-------------------------

struct Vehicle {
    vde_real px, py;
    vde_real vel;
    vde_real yaw;
    vde_real throttle, brake, steer;
};

//-------------------------
// Lifecycle
//-------------------------

Vehicle* vehicle_create() {
    Vehicle* v = (Vehicle*)malloc(sizeof(Vehicle));
    if (!v) return NULL;  // Defensive: check allocation
    
    v->px = v->py = (vde_real)0.0;
    v->vel = (vde_real)0.0;
    v->yaw = (vde_real)0.0;
    v->throttle = v->brake = v->steer = (vde_real)0.0;
    return v;
}

void vehicle_destroy(Vehicle* v) {
    if (!v) return;  // Defensive: check pointer
    free(v);
}

//-------------------------
// Control
//-------------------------

void vehicle_set_inputs(Vehicle* v, vde_real throttle, vde_real brake, vde_real steer) {
    if (!v) return;  // Defensive: validate pointer
    
    v->throttle = vde_clamp(throttle, (vde_real)0.0, (vde_real)1.0);
    v->brake = vde_clamp(brake, (vde_real)0.0, (vde_real)1.0);
    v->steer = vde_clamp(steer, (vde_real)-1.0, (vde_real)1.0);
}

//-------------------------
// Dynamics
//-------------------------

void vehicle_step(Vehicle* v, vde_real dt) {
    if (!v || dt <= (vde_real)0.0) return;  // Defensive: validate args
    
    /* Extremely simple dynamics:
       - throttle increases velocity, brake decreases
       - steer changes yaw, forward motion updates px/py
    */
    vde_real accel = v->throttle * (vde_real)5.0 - v->brake * (vde_real)10.0;
    v->vel += accel * dt;
    if (v->vel < (vde_real)0.0) v->vel = (vde_real)0.0;
    
    v->yaw += v->steer * (vde_real)0.5 * dt;
    
    // Update position
    v->px += v->vel * dt * (vde_real)cos(v->yaw);
    v->py += v->vel * dt * (vde_real)sin(v->yaw);
    
    // Defensive: check for numerical issues
    if (!vde_isfinite(v->px) || !vde_isfinite(v->py) || !vde_isfinite(v->vel) || !vde_isfinite(v->yaw)) {
        // Reset to safe state if we hit numerical issues
        v->px = v->py = (vde_real)0.0;
        v->vel = (vde_real)0.0;
        v->yaw = (vde_real)0.0;
    }
}

//-------------------------
// Telemetry
//-------------------------

void vehicle_write_telemetry(Vehicle* v, vde_real* buffer, int len) {
    if (!v || !buffer || len < 4) return;  // Defensive: validate args
    
    buffer[0] = v->px;
    buffer[1] = v->py;
    buffer[2] = v->vel;
    buffer[3] = v->yaw;
}