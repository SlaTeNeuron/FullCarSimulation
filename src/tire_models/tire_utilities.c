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
    vde_real abs_vehicle_velocity = vde_abs(vehicle_velocity);
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
    vde_real abs_long_velocity = vde_abs(longitudinal_velocity);
    
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
    
    // First apply camber rotation (roll about longitudinal/X axis)
    // Rotation matrix for roll (about X-axis):
    // [1,     0,           0      ]
    // [0, cos(γ),    -sin(γ)      ]
    // [0, sin(γ),     cos(γ)      ]
    vde_real cos_camber = cos(camber_angle);
    vde_real sin_camber = sin(camber_angle);
    
    vde_vec3 after_camber;
    after_camber.x = tire_force->x;
    after_camber.y = tire_force->y * cos_camber - tire_force->z * sin_camber;
    after_camber.z = tire_force->y * sin_camber + tire_force->z * cos_camber;
    
    // Then apply steering rotation (yaw about vertical/Z axis)
    // Rotation matrix for yaw (about Z-axis):
    // [cos(δ), -sin(δ), 0]
    // [sin(δ),  cos(δ), 0]
    // [0,          0,   1]
    vde_real cos_steer = cos(steer_angle);
    vde_real sin_steer = sin(steer_angle);
    
    out_world_force->x = after_camber.x * cos_steer - after_camber.y * sin_steer;
    out_world_force->y = after_camber.x * sin_steer + after_camber.y * cos_steer;
    out_world_force->z = after_camber.z;
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
    
    // Vector from wheel center to contact point (straight down)
    // In tire frame: r = [0, 0, -wheel_radius]
    vde_vec3 r_contact;
    r_contact.x = (vde_real)0.0;
    r_contact.y = (vde_real)0.0;
    r_contact.z = -wheel_radius;
    
    // Angular velocity vector (wheel spins about Y-axis in tire frame)
    // omega = [0, angular_velocity, 0]
    vde_vec3 omega;
    omega.x = (vde_real)0.0;
    omega.y = angular_velocity;
    omega.z = (vde_real)0.0;
    
    // Compute omega × r_contact
    vde_vec3 omega_cross_r;
    vde_vec3_cross(&omega_cross_r, &omega, &r_contact);
    
    // V_contact = V_center - omega × r
    vde_vec3_sub(out_contact_velocity, wheel_center_velocity, &omega_cross_r);
}
