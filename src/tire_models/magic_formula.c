#include "tire_models/magic_formula.h"
#include <stdlib.h>
#include <math.h>

//-------------------------
// Internal State
//-------------------------

struct MagicFormulaTireModel {
    MagicFormulaParams params;
};

//-------------------------
// Lifecycle
//-------------------------

/**
 * Create Magic Formula tire model
 * 
 * Guiggiani Reference: Section 2.10
 *   Magic Formula: y(x) = D * sin[C * arctan{B*x - E*(B*x - arctan(B*x))}]
 * 
 * Output:
 *   - Returns pointer to MagicFormulaTireModel, or NULL on failure
 */
MagicFormulaTireModel* magic_formula_create(void) {
    MagicFormulaTireModel* model = (MagicFormulaTireModel*)malloc(sizeof(MagicFormulaTireModel));
    if (!model) return NULL;
    
    // Initialize with default parameters
    // These are placeholder values - should be tuned from tire test data
    // (Typically obtained from tire manufacturer or testing facility)
    
    // LONGITUDINAL FORCE COEFFICIENTS (b-series)
    // Used in: Fx = f(slip_ratio, Fz)
    // Magic Formula form: y(x) = D*sin[C*arctan{B*x - E*(B*x - arctan(B*x))}]
    model->params.b0 = (vde_real)1.5;       // Shape factor C for longitudinal
    model->params.b1 = (vde_real)0.0;       // Load influence on Dx (peak force) - linear term
    model->params.b2 = (vde_real)1000.0;    // Load influence on Dx - constant term
    model->params.b3 = (vde_real)0.0;       // Load influence on BCD (stiffness) - quadratic term
    model->params.b4 = (vde_real)100.0;     // Load influence on BCD - linear term
    model->params.b5 = (vde_real)0.0;       // Load influence on BCD - exponential decay
    model->params.b6 = (vde_real)0.0;       // Curvature factor E - load squared term
    model->params.b7 = (vde_real)0.0;       // Curvature factor E - load linear term
    model->params.b8 = (vde_real)0.0;       // Curvature factor E - constant term
    model->params.b9 = (vde_real)0.0;       // (Reserved for future use / horizontal shift)
    model->params.b10 = (vde_real)0.0;      // (Reserved for future use / vertical shift)
    
    // LATERAL FORCE COEFFICIENTS (a-series)
    // Used in: Fy = f(slip_angle, Fz, camber_angle)
    // Magic Formula form: y(x) = D*sin[C*arctan{B*x - E*(B*x - arctan(B*x))}]
    model->params.a0 = (vde_real)1.5;       // Shape factor C for lateral
    model->params.a1 = (vde_real)0.0;       // Load influence on Dy (peak force) - linear term
    model->params.a2 = (vde_real)1000.0;    // Load influence on Dy - constant term
    model->params.a3 = (vde_real)1000.0;    // Cornering stiffness - BCD scaling factor
    model->params.a4 = (vde_real)10.0;      // Load influence on cornering stiffness - sin scaling
    model->params.a5 = (vde_real)0.0;       // Cornering stiffness - load normalization (avoid 0)
    model->params.a6 = (vde_real)0.0;       // Curvature factor E - load linear term
    model->params.a7 = (vde_real)0.0;       // Curvature factor E - constant term
    model->params.a8 = (vde_real)0.0;       // Camber influence on horizontal shift (camber thrust)
    
    // VERTICAL STIFFNESS
    // Used for tire deflection calculation: Fz = k_z * deflection
    model->params.vertical_stiffness = (vde_real)200000.0; // 200 kN/m (typical passenger car)
    
    return model;
}

/**
 * Destroy Magic Formula tire model
 * 
 * Input:
 *   - model: Model to destroy (can be NULL)
 */
void magic_formula_destroy(MagicFormulaTireModel* model) {
    if (!model) return;
    free(model);
}

//-------------------------
// Parameters
//-------------------------

/**
 * Set Magic Formula parameters
 * 
 * Input:
 *   - model: Tire model (must be non-NULL)
 *   - params: Parameters to set (must be non-NULL)
 */
void magic_formula_set_params(MagicFormulaTireModel* model, const MagicFormulaParams* params) {
    if (!model || !params) return;
    model->params = *params;
}

/**
 * Get Magic Formula parameters
 * 
 * Input:
 *   - model: Tire model (must be non-NULL)
 *   - out_params: Output buffer (must be non-NULL)
 * 
 * Output:
 *   - out_params: Filled with current parameters
 */
void magic_formula_get_params(const MagicFormulaTireModel* model, MagicFormulaParams* out_params) {
    if (!model || !out_params) return;
    *out_params = model->params;
}

//-------------------------
// Force Computation
//-------------------------

/**
 * Compute tire forces using Magic Formula
 * 
 * This is the CONSTITUTIVE equation for tire forces.
 * 
 * Guiggiani Reference: Section 2.10
 *   Pacejka's Magic Formula:
 *   y(x) = D * sin[C * arctan{B*x - E*(B*x - arctan(B*x))}]
 *   where:
 *     B = stiffness factor
 *     C = shape factor
 *     D = peak factor (peak force)
 *     E = curvature factor
 *     x = slip variable (σ or α)
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
 *   1. Extract slip ratio and slip angle from state
 *   2. Compute longitudinal force using Magic Formula
 *   3. Compute lateral force using Magic Formula
 *   4. Compute vertical force from normal load
 *   5. Compute aligning moment (Mz)
 *   6. Handle combined slip conditions
 */
// Helper function: Magic Formula core calculation
// y(x) = D * sin[C * arctan{B*x - E*(B*x - arctan(B*x))}]
static vde_real magic_formula_eval(vde_real B, vde_real C, vde_real D, vde_real E, vde_real x) {
    vde_real Bx = B * x;
    vde_real arctan_Bx = atan(Bx);
    vde_real arg = Bx - E * (Bx - arctan_Bx);
    vde_real result = D * sin(C * atan(arg));
    return result;
}

void magic_formula_compute_forces(
    const MagicFormulaTireModel* model,
    const TireState* state,
    TireForces* out_forces
) {
    if (!model || !state || !out_forces) return;
    
    const MagicFormulaParams* p = &model->params;
    
    vde_real Fz = state->normal_load;
    
    //-------------------------
    // Pure Longitudinal Force Fx(σ)
    //-------------------------
    vde_real slip_ratio = state->slip_ratio;
    
    // Longitudinal coefficients (simplified MF)
    // Following Pacejka's Magic Formula structure:
    // Fx = Dx * sin[Cx * arctan{Bx*σ - Ex*(Bx*σ - arctan(Bx*σ))}]
    vde_real Cx = p->b0;  // Shape factor (controls curve shape, typically ~1.5-2.0)
    vde_real Dx = Fz * (p->b1 * Fz + p->b2);  // Peak factor (max force: Dx = μx * Fz)
    vde_real Ex = p->b6 * Fz * Fz + p->b7 * Fz + p->b8;  // Curvature factor (curve sharpness)
    vde_real Kx = Fz * (p->b3 * Fz + p->b4) * exp(-p->b5 * Fz);  // Slip stiffness BCD
    vde_real Bx = Kx / (Cx * Dx + (vde_real)0.001);  // Stiffness factor (B = BCD / (C*D))
    
    // Clamp curvature factor
    if (Ex > (vde_real)1.0) Ex = (vde_real)1.0;
    if (Ex < (vde_real)-1.0) Ex = (vde_real)-1.0;
    
    // Compute longitudinal force
    vde_real Fx = magic_formula_eval(Bx, Cx, Dx, Ex, slip_ratio);
    
    //-------------------------
    // Pure Lateral Force Fy(α)
    //-------------------------
    // Slip angle in radians (Magic Formula can work in either rad or deg)
    // This implementation uses radians throughout
    vde_real slip_angle = state->slip_angle;
    
    // Lateral coefficients (simplified MF)
    // Following Pacejka's Magic Formula structure:
    // Fy = Dy * sin[Cy * arctan{By*α - Ey*(By*α - arctan(By*α))}] + SVy
    vde_real Cy = p->a0;  // Shape factor (controls curve shape, typically ~1.5-2.0)
    vde_real Dy = Fz * (p->a1 * Fz + p->a2);  // Peak factor (max force: Dy = μy * Fz)
    vde_real Ey = p->a6 * Fz + p->a7;  // Curvature factor (curve sharpness, controls peak location)
    vde_real Ky = Fz * (p->a3 * sin(p->a4 * atan(Fz / (p->a5 + (vde_real)0.001))));  // Cornering stiffness BCD
    vde_real By = Ky / (Cy * Dy + (vde_real)0.001);  // Stiffness factor (By = BCD / (Cy*Dy))
    
    // Include camber effects
    vde_real SHy = p->a8 * state->camber_angle;  // Horizontal shift (camber thrust)
    vde_real SVy = (vde_real)0.0;  // Vertical shift (simplified - could include ply-steer)
    
    // Clamp curvature factor
    if (Ey > (vde_real)1.0) Ey = (vde_real)1.0;
    if (Ey < (vde_real)-1.0) Ey = (vde_real)-1.0;
    
    // Compute lateral force
    vde_real alpha_y = slip_angle + SHy;
    vde_real Fy = magic_formula_eval(By, Cy, Dy, Ey, alpha_y) + SVy;
    
    //-------------------------
    // Combined Slip
    //-------------------------
    // Simplified combined slip handling using geometric reduction
    // Full MF would use weighting functions: Gx and Gy
    // This approximation reduces forces when both longitudinal and lateral slip are present
    // Theory: Total friction is limited by tire's friction circle
    vde_real combined_slip = sqrt(slip_ratio * slip_ratio + 
                                   tan(slip_angle) * tan(slip_angle));
    
    if (combined_slip > (vde_real)0.1) {
        // Reduce forces under combined slip (friction ellipse approximation)
        vde_real reduction = (vde_real)1.0 / ((vde_real)1.0 + combined_slip);
        Fx *= reduction;
        Fy *= reduction;
    }
    
    //-------------------------
    // Vertical Force (normal load)
    //-------------------------
    // Vertical force typically equals the normal load for quasi-static conditions
    // Dynamic tire deflection would use: Fz = vertical_stiffness * deflection
    vde_real Fz_tire = Fz;
    
    //-------------------------
    // Aligning Moment Mz
    //-------------------------
    // Self-aligning moment (torque about vertical axis)
    // Caused by lateral force acting at pneumatic trail behind contact patch
    // Full MF would compute pneumatic trail from slip angle and load
    vde_real pneumatic_trail = (vde_real)0.03; // ~3cm typical for passenger car tire
    vde_real Mz = -Fy * pneumatic_trail; // Negative for proper sign convention (RHR)
    
    //-------------------------
    // Output Forces
    //-------------------------
    // Forces in tire coordinate frame:
    // X = longitudinal (forward)
    // Y = lateral (left)
    // Z = vertical (up)
    out_forces->force.x = Fx;
    out_forces->force.y = Fy;
    out_forces->force.z = -Fz_tire;  // Negative because force on tire is downward
    
    // Moments about contact patch
    out_forces->moment.x = (vde_real)0.0;  // Overturning moment (simplified)
    out_forces->moment.y = (vde_real)0.0;  // Rolling resistance moment (simplified)
    out_forces->moment.z = Mz;              // Aligning moment
    
    // Store slip values for debugging/telemetry
    out_forces->slip_ratio = slip_ratio;
    out_forces->slip_angle = slip_angle;
}
