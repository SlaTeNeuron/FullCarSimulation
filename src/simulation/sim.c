#include "simulation/simulation_config.h"
#include "vehicle/vehicle.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

//-------------------------
// Internal State
//-------------------------

struct Sim {
    vde_real dt;
    vde_real elapsed_time;
    int running;
    Vehicle* vehicle;
    telemetry_cb_t cb;
    void* cb_user;
    vde_real latest[16];
    int latest_len;
};

//-------------------------
// Lifecycle
//-------------------------

Sim* sim_create(vde_real timestep) {
    if (timestep <= (vde_real)0.0) return NULL;  // Defensive: validate timestep
    
    Sim* s = (Sim*)malloc(sizeof(Sim));
    if (!s) return NULL;  // Defensive: check allocation
    
    s->dt = timestep;
    s->elapsed_time = (vde_real)0.0;
    s->running = 0;
    s->vehicle = vehicle_create();
    if (!s->vehicle) {  // Defensive: check vehicle creation
        free(s);
        return NULL;
    }
    s->cb = NULL;
    s->cb_user = NULL;
    s->latest_len = 4;
    for (int i = 0; i < 16; i++) s->latest[i] = (vde_real)0.0;
    return s;
}

void sim_destroy(Sim* s) {
    if (!s) return;
    vehicle_destroy(s->vehicle);
    free(s);
}

int sim_start(Sim* s) {
    if (!s) return -1;
    s->running = 1;
    return 0;
}

int sim_stop(Sim* s) {
    if (!s) return -1;
    s->running = 0;
    return 0;
}

//-------------------------
// Control
//-------------------------

int sim_step(Sim* s) {
    if (!s || !s->running) return -1;
    if (!s->vehicle) return -1;  // Defensive: check vehicle exists
    
    s->elapsed_time += s->dt;
    vehicle_step(s->vehicle, s->dt);
    
    /* write telemetry */
    vehicle_write_telemetry(s->vehicle, s->latest, s->latest_len);
    
    /* call callback if registered */
    if (s->cb) {
        s->cb(s->latest, s->latest_len, s->cb_user);
    }
    return 0;
}

void sim_set_inputs(Sim* s, vde_real throttle, vde_real brake, vde_real steer) {
    if (!s || !s->vehicle) return;  // Defensive: validate pointers
    
    // Clamp inputs to valid range
    throttle = vde_clamp(throttle, (vde_real)-1.0, (vde_real)1.0);
    brake = vde_clamp(brake, (vde_real)-1.0, (vde_real)1.0);
    steer = vde_clamp(steer, (vde_real)-1.0, (vde_real)1.0);
    
    vehicle_set_inputs(s->vehicle, throttle, brake, steer);
}

//-------------------------
// Telemetry
//-------------------------

void sim_register_telemetry_cb(Sim* s, telemetry_cb_t cb, void* user) {
    if (!s) return;
    s->cb = cb;
    s->cb_user = user;
}

int sim_get_latest_telemetry(Sim* s, vde_real* out_buffer, int max_len) {
    if (!s || !out_buffer || max_len <= 0) return -1;  // Defensive: validate args
    
    int copy = s->latest_len < max_len ? s->latest_len : max_len;
    memcpy(out_buffer, s->latest, sizeof(vde_real) * copy);
    return copy;
}

//-------------------------
// State Export
//-------------------------

void sim_get_state(Sim* s, SimState* out) {
    if (!s || !out) return;
    
    /* Fill SimState from latest telemetry and simulation data */
    out->time = s->elapsed_time;
    
    /* Position (px, py from latest telemetry, pz = 0 for 2D sim) */
    out->px = s->latest_len > 0 ? s->latest[0] : (vde_real)0.0;
    out->py = s->latest_len > 1 ? s->latest[1] : (vde_real)0.0;
    out->pz = (vde_real)0.0;
    
    /* Velocity and acceleration */
    out->vel = s->latest_len > 2 ? s->latest[2] : (vde_real)0.0;
    out->accel = (vde_real)0.0;  /* Not currently tracked; could be computed from velocity change */
    
    /* Orientation (yaw from telemetry, pitch/roll = 0 for 2D sim) */
    out->yaw = s->latest_len > 3 ? s->latest[3] : (vde_real)0.0;
    out->pitch = (vde_real)0.0;
    out->roll = (vde_real)0.0;
    
    /* Vehicle controls (not currently exposed in telemetry) */
    out->steeringAngle = (vde_real)0.0;  /* Could add vehicle_get_steer() if needed */
    out->motorRPM = (vde_real)0.0;       /* Not currently simulated */
}
