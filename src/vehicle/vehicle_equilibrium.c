#include "vehicle/vehicle_equilibrium.h"
#include "vehicle/vehicle_model.h"  // For LoadTransfers definition
#include "vehicle/vehicle.h"
#include "core/math/math_base.h"
#include "core/math/vec3.h"
#include "core/math/mat3.h"

//-------------------------
// Force Assembly (Guiggiani Section 3.5)
//-------------------------

/**
 * Assemble all forces and moments acting on vehicle
 * 
 * Guiggiani Reference: Section 3.5
 *   Forces include: weight, tire forces, aerodynamics
 *   Moments include: tire aligning moments, aero moments, moments from forces
 * 
 * This is the THIRD step in the three-equation structure:
 *   1. CONGRUENCE (compute slips) →
 *   2. CONSTITUTIVE (compute forces from slips) →
 *   3. EQUILIBRIUM (assemble and solve force balance)
 * 
 * Input:
 *   - vehicle: Vehicle pointer (must be non-NULL)
 *   - out_forces: Output structure (must be non-NULL)
 * 
 * Output:
 *   - out_forces: All forces and moments assembled
 */
VDE_API void vehicle_equilibrium_assemble_forces(
    const Vehicle* vehicle,
    VehicleForces* out_forces
) {
    if (!vehicle || !out_forces) return;
    
    // TODO: Implement force assembly
    // 1. Get weight force (m*g downward in world frame, transform to body)
    // 2. Get aerodynamic forces from constitutive
    // 3. Get tire forces from constitutive (all 4 wheels)
    // 4. Sum up total force
    // 5. Compute moments about CG from:
    //    - Tire forces at contact points (position × force)
    //    - Tire aligning moments
    //    - Aero moments
    // 6. Sum up total moment
    
    // Placeholder: zero everything
    out_forces->weight = vde_vec3_zero();
    out_forces->aero = vde_vec3_zero();
    for (int i = 0; i < 4; i++) {
        out_forces->tire_forces[i] = vde_vec3_zero();
        out_forces->tire_moments[i] = vde_vec3_zero();
    }
    out_forces->total_force = vde_vec3_zero();
    out_forces->aero_moment = vde_vec3_zero();
    out_forces->total_moment = vde_vec3_zero();
}

//-------------------------
// Load Transfers (Guiggiani Section 3.7)
//-------------------------

/**
 * Compute load transfers
 * 
 * Guiggiani Reference: Section 3.7
 *   Load transfers occur due to:
 *   - Longitudinal acceleration (Section 3.7.1): ΔFz_long = (m*ax*h) / L
 *   - Lateral acceleration (Section 3.7.2): ΔFz_lat = (m*ay*h) / t
 *   where h = CG height, L = wheelbase, t = track width
 * 
 * Input:
 *   - vehicle: Vehicle pointer (must be non-NULL)
 *   - forces: Assembled forces (must be non-NULL)
 *   - out_transfers: Output structure (must be non-NULL)
 * 
 * Output:
 *   - out_transfers: Load transfer values and vertical loads
 */
VDE_API void vehicle_equilibrium_compute_load_transfers(
    const Vehicle* vehicle,
    const VehicleForces* forces,
    LoadTransfers* out_transfers
) {
    if (!vehicle || !forces || !out_transfers) return;
    
    /* ALPHA STATUS: Using planar model (Option A).
     * Captures basic load transfers for alpha functionality.
     * 
     * POST-ALPHA TODO (Section 3.10):
     * Implement full sprung/unsprung mass system:
     * 1. Separate sprung mass (chassis) dynamics
     * 2. Four unsprung masses (wheel assemblies)
     * 3. Roll dynamics (Section 3.8.11)
     * 4. Pitch dynamics
     * 5. Suspension deflection coupling
     * 6. Proper spring force -> load transfer path
     * 
     * Requires:
     * - Expanded Vehicle structure with SprungMass/UnsprungMass components
     * - Roll angle state variable
     * - Suspension deflection state variables
     * - Equations from Sections 3.8.11-3.8.12, 3.10
     */
    
    // Simplified load transfer (Option A)
    vde_real mass = (vde_real)1500.0;  // kg
    vde_real wheelbase = (vde_real)2.7;  // m
    vde_real track_width = (vde_real)1.5;  // m
    vde_real cg_height = (vde_real)0.5;  // m
    vde_real weight_distribution_front = (vde_real)0.5;  // 50% front
    
    // Static weights
    vde_real weight_total = mass * (vde_real)9.81;
    vde_real weight_front = weight_total * weight_distribution_front;
    vde_real weight_rear = weight_total * ((vde_real)1.0 - weight_distribution_front);
    
    // Longitudinal load transfer (Section 3.7.1)
    // ΔFz = (m * ax * h) / L
    vde_real ax = (vde_real)0.0;  // TODO: Get from accelerations
    out_transfers->longitudinal = (mass * ax * cg_height) / wheelbase;
    
    // Lateral load transfer (Section 3.7.2)
    // ΔFz_front = (m * ay * h * Kφ_front / (Kφ_front + Kφ_rear)) / t_front
    // ΔFz_rear  = (m * ay * h * Kφ_rear  / (Kφ_front + Kφ_rear)) / t_rear
    vde_real ay = (vde_real)0.0;  // TODO: Get from accelerations
    
    // Roll stiffness distribution (typically 50-60% front for understeer)
    // ALPHA: Hardcoded values, not stored in LoadTransfers
    vde_real front_roll_stiffness_fraction = (vde_real)0.55;
    vde_real rear_roll_stiffness_fraction = (vde_real)0.45;
    
    vde_real lateral_load_transfer_total = (mass * ay * cg_height) / track_width;
    out_transfers->lateral_front = lateral_load_transfer_total * front_roll_stiffness_fraction;
    out_transfers->lateral_rear = lateral_load_transfer_total * rear_roll_stiffness_fraction;
    
    // Vertical loads at each tire (Section 3.7.3)
    vde_real Fz_front_static = weight_front * (vde_real)0.5;
    vde_real Fz_rear_static = weight_rear * (vde_real)0.5;
    
    // FL (front left)
    out_transfers->vertical_loads[0] = Fz_front_static 
        - out_transfers->longitudinal * (vde_real)0.5
        - out_transfers->lateral_front;
    
    // FR (front right)
    out_transfers->vertical_loads[1] = Fz_front_static 
        - out_transfers->longitudinal * (vde_real)0.5
        + out_transfers->lateral_front;
    
    // RL (rear left)
    out_transfers->vertical_loads[2] = Fz_rear_static 
        + out_transfers->longitudinal * (vde_real)0.5
        - out_transfers->lateral_rear;
    
    // RR (rear right)
    out_transfers->vertical_loads[3] = Fz_rear_static 
        + out_transfers->longitudinal * (vde_real)0.5
        + out_transfers->lateral_rear;
}

//-------------------------
// Equations of Motion (Guiggiani Section 3.6, 3.11)
//-------------------------

/**
 * Build equations of motion
 * 
 * Guiggiani Reference: Section 3.6
 *   Linear momentum:
 *     m * (u_dot - v*r + w*q) = X
 *     m * (v_dot - w*p + u*r) = Y
 *     m * (w_dot - u*q + v*p) = Z
 *   Angular momentum:
 *     Ix*p_dot - (Iy-Iz)*q*r = L
 *     Iy*q_dot - (Iz-Ix)*r*p = M
 *     Iz*r_dot - (Ix-Iy)*p*q = N
 * 
 * Input:
 *   - vehicle: Vehicle pointer (must be non-NULL)
 *   - forces: Assembled forces (must be non-NULL)
 *   - out_eom: Output structure (must be non-NULL)
 * 
 * Output:
 *   - out_eom: Equations of motion ready to solve
 */
VDE_API void vehicle_equilibrium_build_equations(
    const Vehicle* vehicle,
    const VehicleForces* forces,
    EquationsOfMotion* out_eom
) {
    if (!vehicle || !forces || !out_eom) return;
    
    // Mass properties
    out_eom->mass = (vde_real)1500.0;  // kg
    
    // Inertia tensor (typical sedan, kg*m²)
    out_eom->inertia_tensor = vde_mat3_make(
        (vde_real)500.0,  (vde_real)0.0,    (vde_real)0.0,    // Ixx (roll)
        (vde_real)0.0,    (vde_real)2500.0, (vde_real)0.0,    // Iyy (pitch)
        (vde_real)0.0,    (vde_real)0.0,    (vde_real)2800.0  // Izz (yaw)
    );
    
    // Copy forces directly (already in body frame)
    out_eom->force_x = forces->total_force.x;
    out_eom->force_y = forces->total_force.y;
    out_eom->force_z = forces->total_force.z;
    
    // Copy moments
    out_eom->moment_l = forces->total_moment.x;
    out_eom->moment_m = forces->total_moment.y;
    out_eom->moment_n = forces->total_moment.z;
}

/**
 * Solve for accelerations
 * 
 * Guiggiani Reference: Section 3.6
 *   Solve the 6DOF equations for accelerations:
 *   u_dot, v_dot, w_dot, p_dot, q_dot, r_dot
 * 
 * Input:
 *   - eom: Equations of motion (must be non-NULL)
 *   - out_linear_accel: Output linear acceleration (must be non-NULL)
 *   - out_angular_accel: Output angular acceleration (must be non-NULL)
 * 
 * Output:
 *   - out_linear_accel: Body-frame linear acceleration (u_dot, v_dot, w_dot)
 *   - out_angular_accel: Body-frame angular acceleration (p_dot, q_dot, r_dot)
 */
VDE_API void vehicle_equilibrium_solve_accelerations(
    const EquationsOfMotion* eom,
    vde_vec3* out_linear_accel,
    vde_vec3* out_angular_accel
) {
    if (!eom || !out_linear_accel || !out_angular_accel) return;
    
    /* Implements full 6DOF with Coriolis/centrifugal terms.
     * Requires vehicle velocities (u,v,w,p,q,r) from expanded Vehicle structure.
     * For alpha: structure in place, needs Vehicle expansion.
     */
    
    // Linear accelerations WITH velocity coupling (Guiggiani Section 3.6)
    // Full 6DOF equations:
    //   u_dot = X/m + v*r - w*q
    //   v_dot = Y/m + w*p - u*r
    //   w_dot = Z/m + u*q - v*p
    
    // TODO: Get current velocities from expanded Vehicle structure
    vde_real u = (vde_real)0.0;  // Longitudinal velocity (body frame)
    vde_real v = (vde_real)0.0;  // Lateral velocity (body frame)
    vde_real w = (vde_real)0.0;  // Vertical velocity (body frame)
    vde_real p = (vde_real)0.0;  // Roll rate
    vde_real q = (vde_real)0.0;  // Pitch rate
    vde_real r = (vde_real)0.0;  // Yaw rate
    
    out_linear_accel->x = eom->force_x / eom->mass + v*r - w*q;
    out_linear_accel->y = eom->force_y / eom->mass + w*p - u*r;
    out_linear_accel->z = eom->force_z / eom->mass + u*q - v*p;
    
    // Angular accelerations (simplified - no inertia coupling)
    // Full form: p_dot = (L + (Iy-Iz)*q*r) / Ix
    vde_real Ix = eom->inertia_tensor.m00;  // Diagonal element [0,0]
    vde_real Iy = eom->inertia_tensor.m11;  // Diagonal element [1,1]
    vde_real Iz = eom->inertia_tensor.m22;  // Diagonal element [2,2]
    
    // Avoid division by zero
    if (Ix < (vde_real)1.0) Ix = (vde_real)1.0;
    if (Iy < (vde_real)1.0) Iy = (vde_real)1.0;
    if (Iz < (vde_real)1.0) Iz = (vde_real)1.0;
    
    out_angular_accel->x = eom->moment_l / Ix;
    out_angular_accel->y = eom->moment_m / Iy;
    out_angular_accel->z = eom->moment_n / Iz;
    
    // Sanity checks
    if (!vde_isfinite(out_linear_accel->x)) out_linear_accel->x = (vde_real)0.0;
    if (!vde_isfinite(out_linear_accel->y)) out_linear_accel->y = (vde_real)0.0;
    if (!vde_isfinite(out_linear_accel->z)) out_linear_accel->z = (vde_real)0.0;
    if (!vde_isfinite(out_angular_accel->x)) out_angular_accel->x = (vde_real)0.0;
    if (!vde_isfinite(out_angular_accel->y)) out_angular_accel->y = (vde_real)0.0;
    if (!vde_isfinite(out_angular_accel->z)) out_angular_accel->z = (vde_real)0.0;
}

//-------------------------
// Sprung/Unsprung Mass System (Guiggiani Section 3.10)
//-------------------------

/**
 * Equilibrium for sprung mass (chassis)
 * 
 * Guiggiani Reference: Section 3.10
 *   Sprung mass equilibrium includes suspension forces
 * 
 * Input:
 *   - vehicle: Vehicle pointer (must be non-NULL)
 *   - suspension_forces: Array of 4 suspension forces (must be non-NULL)
 *   - out_linear_accel: Output linear acceleration (must be non-NULL)
 *   - out_angular_accel: Output angular acceleration (must be non-NULL)
 * 
 * Output:
 *   - Accelerations of sprung mass
 */
VDE_API void vehicle_equilibrium_sprung_mass(
    const Vehicle* vehicle,
    vde_vec3* out_force,
    vde_vec3* out_moment
) {
    if (!vehicle || !out_force || !out_moment) return;
    
    // TODO: Implement sprung mass equilibrium
    // 1. Sum suspension forces
    // 2. Add weight, aero forces
    // 3. Solve for sprung mass accelerations
    
    // Placeholder
    *out_force = vde_vec3_zero();
    *out_moment = vde_vec3_zero();
}

/**
 * Equilibrium for unsprung masses (wheels)
 * 
 * Guiggiani Reference: Section 3.10
 *   Each unsprung mass has equilibrium with tire and suspension forces
 * 
 * Input:
 *   - vehicle: Vehicle pointer (must be non-NULL)
 *   - corner_index: 0=FL, 1=FR, 2=RL, 3=RR
 *   - tire_force: Tire force (must be non-NULL)
 *   - suspension_force: Suspension force (must be non-NULL)
 *   - out_accel: Output acceleration (must be non-NULL)
 * 
 * Output:
 *   - Acceleration of unsprung mass
 */
VDE_API void vehicle_equilibrium_unsprung_mass(
    const Vehicle* vehicle,
    int corner_index,
    vde_vec3* out_force,
    vde_vec3* out_moment
) {
    if (!vehicle || !out_force || !out_moment) return;
    if (corner_index < 0 || corner_index >= 4) return;
    
    // TODO: Implement unsprung mass equilibrium
    // F = m_unsprung * a
    // F = F_tire - F_suspension - m_unsprung*g
    
    // Placeholder
    *out_force = vde_vec3_zero();
    *out_moment = vde_vec3_zero();
}

//-------------------------
// Roll Motion (Guiggiani Section 3.8.11, Chapter 9)
//-------------------------

/**
 * Compute roll angle and roll rate
 * 
 * Guiggiani Reference: Section 3.8.11, Chapter 9
 *   Roll angle from lateral acceleration and suspension stiffness
 * 
 * Input:
 *   - vehicle: Vehicle pointer (must be non-NULL)
 *   - lateral_accel: Lateral acceleration (m/s²)
 *   - out_roll_angle: Output roll angle (rad) (must be non-NULL)
 *   - out_roll_rate: Output roll rate (rad/s) (must be non-NULL)
 * 
 * Output:
 *   - Roll angle and rate
 */
VDE_API void vehicle_equilibrium_compute_roll(
    const Vehicle* vehicle,
    vde_real lateral_accel,
    vde_real* out_roll_angle,
    vde_real* out_roll_rate
) {
    if (!vehicle || !out_roll_angle || !out_roll_rate) return;
    
    // TODO: Implement roll dynamics
    // φ = (m*h*ay) / Kφ
    // where Kφ = roll stiffness
    
    // Placeholder
    *out_roll_angle = (vde_real)0.0;
    *out_roll_rate = (vde_real)0.0;
}
