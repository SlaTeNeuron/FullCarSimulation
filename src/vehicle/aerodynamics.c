#include "vehicle/aerodynamics.h"
#include "vehicle/vehicle.h"
#include <stdlib.h>

//-------------------------
// Internal State
//-------------------------

struct Aerodynamics {
    vde_real drag_coeff;    // Cd
    vde_real lift_coeff;    // Cl (negative for downforce)
    vde_real frontal_area;  // m²
    vde_real air_density;   // kg/m³
    vde_real aero_balance;  // front downforce fraction [0, 1]
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create aerodynamics component
 * 
 * Guiggiani Reference: Section 7.6
 *   Drag = 0.5 * ρ * A * Cd * V²
 *   Downforce = 0.5 * ρ * A * Cl * V²
 * 
 * Output:
 *   - Returns pointer to Aerodynamics, or NULL on failure
 */
Aerodynamics* aerodynamics_create(void) {
    Aerodynamics* aero = (Aerodynamics*)malloc(sizeof(Aerodynamics));
    if (!aero) return NULL;
    
    aero->drag_coeff    = (vde_real)1.35;   // TBRe full-aero setup
    aero->lift_coeff    = (vde_real)-2.10;  // strong downforce
    aero->frontal_area  = (vde_real)0.92;   // m²
    aero->air_density   = (vde_real)1.225;  // sea level, 15 °C
    aero->aero_balance  = (vde_real)0.40;   // 40 % front downforce
    
    return aero;
}

/**
 * Destroy aerodynamics component
 * 
 * Input:
 *   - aero: Aerodynamics to destroy (can be NULL)
 */
void aerodynamics_destroy(Aerodynamics* aero) {
    if (!aero) return;
    free(aero);
}

//-------------------------
// Properties
//-------------------------

/**
 * Set drag coefficient
 * 
 * Input:
 *   - aero: Aerodynamics component (must be non-NULL)
 *   - cd: Drag coefficient (typically 0.2-0.4 for cars)
 */
void aerodynamics_set_drag_coeff(Aerodynamics* aero, vde_real cd) {
    if (!aero) return;
    aero->drag_coeff = cd;
}

/**
 * Set lift coefficient (negative for downforce)
 * 
 * Input:
 *   - aero: Aerodynamics component (must be non-NULL)
 *   - cl: Lift coefficient (negative for downforce, e.g., -2.0 for Formula car)
 */
void aerodynamics_set_lift_coeff(Aerodynamics* aero, vde_real cl) {
    if (!aero) return;
    aero->lift_coeff = cl;
}

/**
 * Set frontal area
 * 
 * Input:
 *   - aero: Aerodynamics component (must be non-NULL)
 *   - area: Frontal area in m² (typically 2.0-2.5 for cars)
 */
void aerodynamics_set_frontal_area(Aerodynamics* aero, vde_real area) {
    if (!aero) return;
    aero->frontal_area = area;
}

/** Set air density (kg/m³). Standard sea-level value: 1.225. */
void aerodynamics_set_air_density(Aerodynamics* aero, vde_real rho) {
    if (!aero) return;
    aero->air_density = rho;
}

vde_real aerodynamics_get_air_density(const Aerodynamics* aero) {
    if (!aero) return (vde_real)1.225;
    return aero->air_density;
}

/** Set fraction of total downforce acting on the front axle [0, 1]. */
void aerodynamics_set_aero_balance(Aerodynamics* aero, vde_real balance) {
    if (!aero) return;
    aero->aero_balance = vde_clamp(balance, (vde_real)0.0, (vde_real)1.0);
}

vde_real aerodynamics_get_aero_balance(const Aerodynamics* aero) {
    if (!aero) return (vde_real)0.5;
    return aero->aero_balance;
}

//-------------------------
// Force Computation
//-------------------------

/**
 * Compute aerodynamic forces and moments
 * 
 * Guiggiani Reference: Section 7.6
 *   F_drag = 0.5 * ρ * A * Cd * V²
 *   F_lift = 0.5 * ρ * A * Cl * V² (negative for downforce)
 * 
 * Input:
 *   - aero: Aerodynamics component (must be non-NULL)
 *   - vehicle: Vehicle state for velocity (must be non-NULL)
 *   - out_forces: Output structure (must be non-NULL)
 * 
 * Output:
 *   - out_forces: Filled with aerodynamic force and moment vectors
 * 
 * Functionality:
 *   1. Get vehicle velocity
 *   2. Compute dynamic pressure: q = 0.5 * ρ * V²
 *   3. Compute drag force (opposite to velocity)
 *   4. Compute lift/downforce (vertical)
 *   5. Compute moments due to COP offset
 */
void aerodynamics_compute_forces(const Aerodynamics* aero, const Vehicle* vehicle, AeroForces* out_forces) {
    if (!aero || !vehicle || !out_forces) return;
    
    // TODO: Implement aerodynamic force computation
    // 1. Get vehicle velocity from vehicle
    // 2. Compute speed squared
    // 3. Compute drag force: F_drag = -0.5 * ρ * A * Cd * V² * (V_hat)
    // 4. Compute downforce: F_down = 0.5 * ρ * A * Cl * V² * (up vector)
    // 5. Compute moments if needed
    
    // Placeholder: zero forces
    out_forces->force = vde_vec3_zero();
    out_forces->moment = vde_vec3_zero();
}
