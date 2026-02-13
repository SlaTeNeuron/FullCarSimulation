#pragma once
// Vehicle Dynamics Engine - Vehicle Congruence (Kinematic) Equations
// Guiggiani Chapter 3, Section 3.2: "Vehicle Congruence Equations"
//
// The CONGRUENCE equations describe the kinematic relationships within
// the vehicle - how different parts move relative to each other.
//
// This is the FIRST equation set in Guiggiani's three-equation structure:
//   1. CONGRUENCE → 2. CONSTITUTIVE → 3. EQUILIBRIUM
//
// Key Sections:
//   - 3.2.1: Vehicle velocities (u, v, r)
//   - 3.2.2: Velocities at wheel contact points
//   - 3.2.7: Tire kinematics (computing tire slips from motion)
//   - Chapter 5: Detailed kinematics of cornering

#include "core/math/math_base.h"
#include "core/math/vec3.h"
#include "core/math/quat.h"

//-------------------------
// Forward Declarations
//-------------------------

typedef struct Vehicle Vehicle;

//-------------------------
// Vehicle Velocity State
// (Guiggiani Section 3.2.1)
//-------------------------

// Vehicle velocities in body-fixed frame
typedef struct VehicleVelocities {
    vde_real u;              // Longitudinal velocity (m/s) - forward
    vde_real v;              // Lateral velocity (m/s) - sideways
    vde_real w;              // Vertical velocity (m/s) - up/down
    vde_real r;              // Yaw rate (rad/s) - about vertical axis
    vde_real p;              // Roll rate (rad/s) - about longitudinal axis
    vde_real q;              // Pitch rate (rad/s) - about lateral axis
} VehicleVelocities;

//-------------------------
// Wheel Contact Point Velocities
// (Guiggiani Section 3.2.2)
//-------------------------

// Velocity at each wheel contact point
typedef struct WheelContactVelocity {
    vde_vec3 velocity;       // Velocity of contact point (body frame)
    vde_real longitudinal;   // Longitudinal component (m/s)
    vde_real lateral;        // Lateral component (m/s)
    vde_real vertical;       // Vertical component (m/s)
} WheelContactVelocity;

//-------------------------
// Tire Slip Calculations
// (Guiggiani Section 3.2.7 and Chapter 2, Section 2.7)
//-------------------------

// Tire slips computed from kinematics
typedef struct TireSlips {
    vde_real sigma;          // Longitudinal slip ratio (dimensionless)
    vde_real alpha;          // Slip angle (rad)
    vde_real phi;            // Spin slip (rad/m) - camber related
} TireSlips;

//-------------------------
// Congruence Computation API
//-------------------------

// Compute vehicle velocities from state
VDE_API void vehicle_congruence_compute_velocities(
    const Vehicle* vehicle,
    VehicleVelocities* out_velocities
);

// Compute velocity at each wheel contact point (Section 3.2.2)
// corner_index: 0=FL, 1=FR, 2=RL, 3=RR
VDE_API void vehicle_congruence_compute_wheel_velocity(
    const Vehicle* vehicle,
    int corner_index,
    WheelContactVelocity* out_velocity
);

// Compute tire slips from kinematics (Section 3.2.7)
// This connects vehicle motion to tire behavior
VDE_API void vehicle_congruence_compute_tire_slips(
    const Vehicle* vehicle,
    int corner_index,
    TireSlips* out_slips
);

// Compute all tire slips for all four wheels
VDE_API void vehicle_congruence_compute_all_tire_slips(
    const Vehicle* vehicle,
    TireSlips* out_slips  // Array of 4 elements
);

//-------------------------
// Kinematic Centers
// (Guiggiani Chapter 5: Kinematics of Cornering)
//-------------------------

// Velocity center (instantaneous center of rotation)
typedef struct VelocityCenter {
    vde_vec3 position;       // Position of velocity center (vehicle frame)
    vde_real curvature;      // Path curvature (1/m)
} VelocityCenter;

// Compute instantaneous center of rotation (Section 5.1, 5.2)
VDE_API void vehicle_congruence_compute_velocity_center(
    const Vehicle* vehicle,
    VelocityCenter* out_center
);

//-------------------------
// Ackermann Kinematics
// (Guiggiani Section 3.2.3)
//-------------------------

// Compute ideal Ackermann steering angles
VDE_API void vehicle_congruence_compute_ackermann_angles(
    vde_real steer_angle,    // Input: steering wheel angle (rad)
    vde_real wheelbase,      // Vehicle wheelbase (m)
    vde_real track_width,    // Track width (m)
    vde_real* out_inner,     // Output: inner wheel angle (rad)
    vde_real* out_outer      // Output: outer wheel angle (rad)
);

//-------------------------
// Suspension Kinematics
// (Guiggiani Section 3.8)
//-------------------------

// Compute suspension deflection rates from vehicle motion
VDE_API void vehicle_congruence_compute_suspension_rates(
    const Vehicle* vehicle,
    vde_real* out_deflection_rates  // Array of 4 elements (m/s)
);

//-------------------------
// Validation
//-------------------------

// Verify kinematic consistency (e.g., no-slip condition at contact)
VDE_API int vehicle_congruence_validate(const Vehicle* vehicle);

//-------------------------
// Key Formulas (from Guiggiani)
//-------------------------

// Section 3.2.1: Body-fixed velocities
//   u = V * cos(β)
//   v = V * sin(β)
//   where V is speed, β is sideslip angle

// Section 3.2.7: Longitudinal slip ratio
//   σ = (ω * re - Vx) / |Vx|
//   where ω is wheel speed, re is effective radius, Vx is contact point velocity

// Section 3.2.7: Slip angle
//   tan(α) = Vy / |Vx|
//   where Vy, Vx are lateral and longitudinal contact point velocities

