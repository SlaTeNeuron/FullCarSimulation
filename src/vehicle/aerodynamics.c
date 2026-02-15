#include "vehicle/aerodynamics.h"
#include "vehicle/vehicle.h"
#include <stdlib.h>

//-------------------------
// Internal State
//-------------------------

struct Aerodynamics {
    vde_real drag_coeff;      // Drag coefficient Cd
    vde_real lift_coeff;      // Lift coefficient Cl (negative for downforce)
    vde_real frontal_area;    // Frontal area (m²)
    vde_real air_density;     // Air density (kg/m³, typically ~1.225)
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
    
    // Default values for typical road car
    aero->drag_coeff = (vde_real)0.3;
    aero->lift_coeff = (vde_real)0.0;  // Zero downforce by default
    aero->frontal_area = (vde_real)2.0; // ~2 m²
    aero->air_density = (vde_real)1.225; // Sea level
    
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
