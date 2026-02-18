#include "vehicle/tire.h"
#include "tire_models/magic_formula.h"
#include "tire_models/brush_models.h"
#include <stdlib.h>

//-------------------------
// Internal State
//-------------------------

struct Tire {
    vde_real radius;          // Tire radius (m)
    void* tire_model;         // Pointer to tire model (Magic Formula or Brush)
    int model_type;           // 0=Magic Formula, 1=Brush
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create tire component
 * 
 * Guiggiani Reference: Chapter 2
 *   The tire is the critical constitutive element
 * 
 * Output:
 *   - Returns pointer to Tire, or NULL on failure
 */
Tire* tire_create(void) {
    Tire* tire = (Tire*)malloc(sizeof(Tire));
    if (!tire) return NULL;
    
    tire->radius = (vde_real)0.32; // 0.32m typical (640mm diameter)
    
    // Use Brush Model (0 for Magic Formula, 1 for Brush)
    tire->model_type = 1;
    tire->tire_model = brush_tire_create();
    
    if (!tire->tire_model) {
        free(tire);
        return NULL;
    }
    
    return tire;
}

/**
 * Destroy tire component
 * 
 * Input:
 *   - tire: Tire to destroy (can be NULL)
 */
void tire_destroy(Tire* tire) {
    if (!tire) return;
    
    // Destroy tire model based on type
    if (tire->tire_model) {
        if (tire->model_type == 0) {
            magic_formula_destroy((MagicFormulaTireModel*)tire->tire_model);
        } else {
            brush_tire_destroy((BrushTireModel*)tire->tire_model);
        }
    }
    
    free(tire);
}

//-------------------------
// Properties
//-------------------------

/**
 * Set tire radius
 * 
 * Input:
 *   - tire: Tire component (must be non-NULL)
 *   - radius: Tire radius in meters (typically 0.25-0.35m)
 */
void tire_set_radius(Tire* tire, vde_real radius) {
    if (!tire) return;
    tire->radius = radius;
}

/**
 * Get tire radius
 * 
 * Input:
 *   - tire: Tire component (must be non-NULL)
 * 
 * Output:
 *   - Returns tire radius in meters
 */
vde_real tire_get_radius(const Tire* tire) {
    if (!tire) return (vde_real)0.0;
    return tire->radius;
}

//-------------------------
// Force Computation
//-------------------------

/**
 * Compute tire forces from current state
 * 
 * This is the CONSTITUTIVE equation mapping tire slips to forces.
 * 
 * Guiggiani Reference: Chapter 2, Section 2.8, 2.10
 *   Forces = f(slips, load, camber, ...)
 * 
 * Input:
 *   - tire: Tire component (must be non-NULL)
 *   - state: Tire state (slips, load, etc., must be non-NULL)
 *   - out_forces: Output structure (must be non-NULL)
 * 
 * Output:
 *   - out_forces: Filled with computed tire forces and moments
 * 
 * Functionality:
 *   1. Check tire model type
 *   2. Call appropriate tire model (Magic Formula or Brush)
 *   3. Return forces in tire frame
 */
void tire_compute_forces(Tire* tire, const TireState* state, TireForces* out_forces) {
    if (!tire || !state || !out_forces) return;
    
    if (!tire->tire_model) {
        // No model: zero forces
        out_forces->force = vde_vec3_zero();
        out_forces->moment = vde_vec3_zero();
        out_forces->slip_ratio = state->slip_ratio;
        out_forces->slip_angle = state->slip_angle;
        return;
    }
    
    // Call appropriate tire model
    if (tire->model_type == 0) {
        magic_formula_compute_forces((MagicFormulaTireModel*)tire->tire_model, state, out_forces);
    } else {
        brush_tire_compute_forces((BrushTireModel*)tire->tire_model, state, out_forces);
    }
}
