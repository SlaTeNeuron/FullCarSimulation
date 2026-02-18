#include "tire_models/brush_models.h"
#include <stdlib.h>
#include <math.h>

//-------------------------
// Internal State
//-------------------------

struct BrushTireModel {
    BrushTireParams params;
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create brush tire model
 * 
 * Guiggiani Reference: Chapter 10
 *   Physical model based on contact patch bristle deflections
 * 
 * Output:
 *   - Returns pointer to BrushTireModel, or NULL on failure
 */
BrushTireModel* brush_tire_create(void) {
    BrushTireModel* model = (BrushTireModel*)malloc(sizeof(BrushTireModel));
    if (!model) return NULL;
    
    // Default parameters
    model->params.longitudinal_stiffness = (vde_real)100000.0; // 100 kN
    model->params.lateral_stiffness = (vde_real)80000.0;       // 80 kN
    model->params.friction_coeff = (vde_real)1.0;              // μ = 1.0
    model->params.contact_patch_length = (vde_real)0.15;      // 15 cm
    model->params.contact_patch_width = (vde_real)0.20;       // 20 cm
    
    return model;
}

/**
 * Destroy brush tire model
 * 
 * Input:
 *   - model: Model to destroy (can be NULL)
 */
void brush_tire_destroy(BrushTireModel* model) {
    if (!model) return;
    free(model);
}

//-------------------------
// Parameters
//-------------------------

/**
 * Set brush model parameters
 * 
 * Input:
 *   - model: Tire model (must be non-NULL)
 *   - params: Parameters to set (must be non-NULL)
 */
void brush_tire_set_params(BrushTireModel* model, const BrushTireParams* params) {
    if (!model || !params) return;
    model->params = *params;
}

/**
 * Get brush model parameters
 * 
 * Input:
 *   - model: Tire model (must be non-NULL)
 *   - out_params: Output buffer (must be non-NULL)
 * 
 * Output:
 *   - out_params: Filled with current parameters
 */
void brush_tire_get_params(const BrushTireModel* model, BrushTireParams* out_params) {
    if (!model || !out_params) return;
    *out_params = model->params;
}

//-------------------------
// Force Computation
//-------------------------

/**
 * Compute tire forces using brush model
 * 
 * This is a physical CONSTITUTIVE model based on contact patch theory.
 * 
 * Guiggiani Reference: Chapter 10
 *   Brush model divides contact patch into adhesion and sliding zones.
 *   Bristles in adhesion zone are deflected elastically.
 *   Bristles in sliding zone slide at friction limit.
 * 
 * Input:
 *   - model: Tire model (must be non-NULL)
 *   - state: Tire state (slips, load, etc., must be non-NULL)
 *   - out_forces: Output structure (must be non-NULL)
 * 
 * Output:
 *   - out_forces: Filled with computed forces and moments
 * 
 * Functionality:
 *   1. Compute adhesion zone length based on slip
 *   2. Integrate bristle deflections over adhesion zone
 *   3. Add sliding friction from sliding zone
 *   4. Compute lateral forces similarly
 *   5. Handle combined slip with coupled deflections
 *   6. Compute aligning moment from pressure distribution
 */
void brush_tire_compute_forces(
    const BrushTireModel* model,
    const TireState* state,
    TireForces* out_forces
) {
    if (!model || !state || !out_forces) return;
    
    const BrushTireParams* p = &model->params;
    
    vde_real Fz = state->normal_load;
    vde_real sigma = state->slip_ratio;
    vde_real alpha = state->slip_angle;
    
    // Maximum friction force
    vde_real Fmax = p->friction_coeff * Fz;
    
    //-------------------------
    // SIMPLIFIED BRUSH MODEL (Guiggiani Chapter 10)
    //-------------------------
    // The full brush model divides the contact patch into:
    // 1. Adhesion zone (front): Bristles stick and deflect elastically
    // 2. Sliding zone (rear): Bristles slide at friction limit
    //
    // The adhesion zone length 'a' is found by solving:
    //   Elastic force in adhesion zone = Sliding force at transition
    //
    // For FULL IMPLEMENTATION (TODO for Phase 2+):
    //   - Section 10.3: Compute adhesion zone length from slip and stiffness
    //   - Section 10.4-10.5: Integrate bristle deflections
    //   - Section 10.6: Handle combined slip with coupled equations
    //   - Section 10.7: Include spin slip (camber effects)
    //   - Section 10.8: Model transient bristle deflections (dynamic)
    
    // SIMPLIFIED VERSION: Use linear region + saturation
    // This approximates the brush model behavior but without full integration
    
    //-------------------------
    // Longitudinal Force
    //-------------------------
    // In small slip region: Fx ≈ Cx * σ (linear)
    // At high slip: Fx → ±μ*Fz (friction limit)
    vde_real Fx_linear = p->longitudinal_stiffness * sigma;
    vde_real Fx = Fx_linear;
    
    // Saturate at friction limit
    if (Fx > Fmax) Fx = Fmax;
    if (Fx < -Fmax) Fx = -Fmax;
    
    //-------------------------
    // Lateral Force
    //-------------------------
    // In small slip angle: Fy ≈ Cα * α (linear)
    // At high slip angle: Fy → ±μ*Fz (friction limit)
    vde_real Fy_linear = p->lateral_stiffness * alpha;
    vde_real Fy = Fy_linear;
    
    // Saturate at friction limit
    if (Fy > Fmax) Fy = Fmax;
    if (Fy < -Fmax) Fy = -Fmax;
    
    //-------------------------
    // Combined Slip (Friction Circle/Ellipse)
    //-------------------------
    // Total force limited by friction circle
    vde_real F_total = sqrt(Fx * Fx + Fy * Fy);
    if (F_total > Fmax) {
        // Scale both forces to stay within friction limit
        vde_real scale = Fmax / F_total;
        Fx *= scale;
        Fy *= scale;
    }
    
    //-------------------------
    // Vertical Force
    //-------------------------
    vde_real Fz_tire = Fz; // Reaction force equals normal load
    
    //-------------------------
    // Aligning Moment (Self-Aligning Torque)
    //-------------------------
    // Caused by lateral force acting at center of pressure
    // Center of pressure shifts rearward with increasing slip
    // Full model: pneumatic trail = f(adhesion_zone_length)
    // Simplified: Use constant pneumatic trail that decreases with slip
    vde_real slip_angle_deg = vde_abs(alpha) * (vde_real)57.2958;
    vde_real pneumatic_trail = (vde_real)0.03 * exp(-(vde_real)0.1 * slip_angle_deg);
    vde_real Mz = -Fy * pneumatic_trail;
    
    //-------------------------
    // Output Forces
    //-------------------------
    out_forces->force.x = Fx;
    out_forces->force.y = Fy;
    out_forces->force.z = -Fz_tire;
    
    out_forces->moment.x = (vde_real)0.0; // Overturning moment (simplified)
    out_forces->moment.y = (vde_real)0.0; // Rolling resistance (simplified)
    out_forces->moment.z = Mz;
    
    out_forces->slip_ratio = sigma;
    out_forces->slip_angle = alpha;
    
    /* ALPHA STATUS: Using simplified linear+saturation model.
     * This provides good behavior for alpha release.
     * 
     * POST-ALPHA TODO (Guiggiani Chapter 10):
     * 1. Implement adhesion zone calculation (Section 10.3)
     * 2. Add bristle deflection integration (Sections 10.4-10.7)
     * 3. Implement coupled equations for combined slip (Section 10.6)
     * 4. Add transient bristle state variables (Section 10.8):
     *    - Bristle deflection history
     *    - Relaxation length dynamics
     *    - Memory effects in contact patch
     * 
     * For full transient model, brush model struct needs:
     *    - vde_real bristle_state[N];  // Deflection state
     *    - vde_real relaxation_state;  // Transient relaxation
     */
}
