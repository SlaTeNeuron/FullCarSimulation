#include "tire_models/tire_utilities.h"
#include <math.h>

//-------------------------
// Slip Computation (CONGRUENCE)
//-------------------------

/**
 * Compute longitudinal slip ratio
 * 
 * This is a CONGRUENCE equation relating wheel motion to vehicle motion.
 * 
 * Guiggiani Reference: Section 2.7.2
 *   σ = (ω*re - Vx) / |Vx|
 *   where:
 *     ω = wheel angular velocity
 *     re = effective rolling radius
 *     Vx = longitudinal velocity at contact point
 * 
 * Input:
 *   - wheel_angular_velocity: Wheel spin rate in rad/s
 *   - wheel_radius: Effective rolling radius in meters
 *   - vehicle_velocity: Longitudinal velocity in m/s
 * 
 * Output:
 *   - Returns slip ratio (dimensionless)
 * 
 * Functionality:
 *   1. Compute wheel peripheral speed: V_wheel = ω * r
 *   2. Compute slip: σ = (V_wheel - V_vehicle) / |V_vehicle|
 *   3. Handle special case when V_vehicle ≈ 0
 */
vde_real tire_util_compute_slip_ratio(
    vde_real wheel_angular_velocity,
    vde_real wheel_radius,
    vde_real vehicle_velocity
) {
    // Wheel peripheral speed
    vde_real wheel_speed = wheel_angular_velocity * wheel_radius;
    
    // Avoid division by zero
    vde_real abs_vehicle_velocity = fabs(vehicle_velocity);
    if (abs_vehicle_velocity < (vde_real)0.1) {
        // At very low speed, use simplified slip
        return (vde_real)0.0;
    }
    
    // Compute slip ratio
    vde_real slip_ratio = (wheel_speed - vehicle_velocity) / abs_vehicle_velocity;
    
    // Clamp to reasonable range [-1, 10] (locked to heavy spin)
    if (slip_ratio < (vde_real)-1.0) slip_ratio = (vde_real)-1.0;
    if (slip_ratio > (vde_real)10.0) slip_ratio = (vde_real)10.0;
    
    return slip_ratio;
}

/**
 * Compute slip angle from velocity components
 * 
 * This is a CONGRUENCE equation relating velocity components to slip angle.
 * 
 * Guiggiani Reference: Section 2.7.3
 *   tan(α) = Vy / |Vx|
 *   where:
 *     Vy = lateral velocity at contact point
 *     Vx = longitudinal velocity at contact point
 * 
 * Input:
 *   - longitudinal_velocity: Vx in m/s
 *   - lateral_velocity: Vy in m/s
 * 
 * Output:
 *   - Returns slip angle in radians
 * 
 * Functionality:
 *   1. Compute angle: α = atan2(Vy, |Vx|)
 *   2. Handle special case when Vx ≈ 0
 */
vde_real tire_util_compute_slip_angle(
    vde_real longitudinal_velocity,
    vde_real lateral_velocity
) {
    // Use atan2 for proper quadrant handling
    vde_real abs_long_velocity = fabs(longitudinal_velocity);
    
    // At very low speed, slip angle is undefined
    if (abs_long_velocity < (vde_real)0.1) {
        return (vde_real)0.0;
    }
    
    // Compute slip angle
    vde_real slip_angle = atan2(lateral_velocity, abs_long_velocity);
    
    return slip_angle;
}

//-------------------------
// Force Transformations
//-------------------------

/**
 * Transform forces from tire frame to world frame
 * 
 * Input:
 *   - tire_force: Force in tire frame (must be non-NULL)
 *   - steer_angle: Steering angle in radians
 *   - camber_angle: Camber angle in radians
 *   - out_world_force: Output force in world frame (must be non-NULL)
 * 
 * Output:
 *   - out_world_force: Transformed force
 * 
 * Functionality:
 *   1. Apply steering rotation about vertical axis
 *   2. Apply camber rotation about longitudinal axis
 *   3. Return transformed force
 */
void tire_util_transform_forces(
    const vde_vec3* tire_force,
    vde_real steer_angle,
    vde_real camber_angle,
    vde_vec3* out_world_force
) {
    if (!tire_force || !out_world_force) return;
    
    // TODO: Implement proper rotation transformations
    // 1. Rotate by steer angle (yaw)
    // 2. Rotate by camber angle (roll)
    
    // Placeholder: copy through
    *out_world_force = *tire_force;
}

//-------------------------
// Contact Point Kinematics
//-------------------------

/**
 * Compute contact patch velocity
 * 
 * Computes the velocity of the tire contact point given wheel center
 * motion and wheel rotation.
 * 
 * Input:
 *   - wheel_center_velocity: Velocity of wheel center (must be non-NULL)
 *   - wheel_radius: Wheel radius in meters
 *   - angular_velocity: Wheel angular velocity in rad/s
 *   - out_contact_velocity: Output velocity (must be non-NULL)
 * 
 * Output:
 *   - out_contact_velocity: Velocity of contact point
 * 
 * Functionality:
 *   V_contact = V_center - ω × r
 *   where r is the vector from center to contact point
 */
void tire_util_compute_contact_velocity(
    const vde_vec3* wheel_center_velocity,
    vde_real wheel_radius,
    vde_real angular_velocity,
    vde_vec3* out_contact_velocity
) {
    if (!wheel_center_velocity || !out_contact_velocity) return;
    
    // TODO: Implement contact velocity computation
    // V_contact = V_center - omega × r_contact
    
    // Placeholder: copy center velocity
    *out_contact_velocity = *wheel_center_velocity;
}
