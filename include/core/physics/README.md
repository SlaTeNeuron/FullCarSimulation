# Core Physics - Dynamics Solvers and Equations of Motion

This directory contains the core physics infrastructure for vehicle dynamics
simulation, following Guiggiani's methodology.

## Overview

The physics system implements the **EQUILIBRIUM** equations - the third component
of Guiggiani's three-equation structure:

```
1. CONGRUENCE → 2. CONSTITUTIVE → 3. EQUILIBRIUM (THIS DIRECTORY)
```

---

## Modules

### `equations_of_motion.h` - 6DOF Dynamics
**Guiggiani Chapter 3, Sections 3.4-3.6; Chapter 9, Section 9.6**

Builds and solves the vehicle equilibrium equations.

#### Body-Fixed Frame Equations (Section 3.6)

**Linear Momentum:**
```
m * (u_dot - v*r + w*q) = X
m * (v_dot - w*p + u*r) = Y
m * (w_dot - u*q + v*p) = Z
```

**Angular Momentum:**
```
Ix*p_dot - (Iy-Iz)*q*r = L
Iy*q_dot - (Iz-Ix)*r*p = M
Iz*r_dot - (Ix-Iy)*p*q = N
```

where:
- u, v, w = linear velocities (m/s)
- p, q, r = angular velocities (rad/s)
- X, Y, Z = forces (N)
- L, M, N = moments (Nm)
- Ix, Iy, Iz = principal inertias (kg·m²)

#### Planar (3DOF) vs Full 3D (6DOF)

**Planar Model** (Chapters 3, 6, 7):
- Motion in horizontal plane only
- 3 DOFs: longitudinal (x), lateral (y), yaw (ψ)
- Simplified for handling analysis
- Sufficient for many applications

**Full 3D Model** (Chapter 9):
- Complete 6DOF rigid body
- Includes roll, pitch, vertical motion
- Required for ride analysis
- More realistic but more complex

---

### `dynamics_solver.h` - Integration and Solving
**Guiggiani Chapter 3, Section 3.11**

Main solver that orchestrates the complete three-equation model:

```c
void dynamics_solver_solve(DynamicsSolver* solver, Vehicle* vehicle, vde_real dt) {
    // 1. Compute CONGRUENCE: kinematics, tire slips
    vehicle_model_compute_congruence(vehicle);
    
    // 2. Evaluate CONSTITUTIVE: tire forces, spring forces, etc.
    vehicle_model_compute_constitutive(vehicle);
    
    // 3. Assemble and solve EQUILIBRIUM: F=ma, M=Iα
    vehicle_model_assemble_equilibrium(vehicle, accelerations);
    
    // 4. Integrate state forward in time
    integrator_step(vehicle, accelerations, dt);
}
```

---

### `constraints.h` - Constraint Solver
**Guiggiani Section 3.8 (suspension kinematics)**

Handles kinematic constraints:
- Ground contact constraints
- Suspension kinematic linkages
- Wheel assembly constraints

Not heavily emphasized in Guiggiani (he focuses on unconstrained dynamics),
but useful for complex suspension geometries.

---

## Integration with Vehicle Model

### Three-Equation Model Execution

The physics solver ties together all three equation types:

#### Step 1: CONGRUENCE (Kinematics)
From `vehicle/vehicle_congruence.h`:
- Compute vehicle velocities at each wheel
- Calculate tire slips (σ, α, φ)
- Compute suspension deflection rates

#### Step 2: CONSTITUTIVE (Component Behavior)
From `vehicle/vehicle_constitutive.h` and `tire_models/`:
- Evaluate tire forces from slips
- Compute spring/damper forces
- Calculate aerodynamic forces
- Determine brake torques

#### Step 3: EQUILIBRIUM (Force Balance) - THIS DIRECTORY
- Assemble all forces and moments
- Compute load transfers
- Build mass matrix and force vector
- Solve for accelerations

#### Step 4: INTEGRATION
From `core/integrator/`:
- Update velocities from accelerations
- Update positions from velocities
- Maintain quaternion normalization

---

## Mathematical Structure

### Generalized Coordinates (Section 3.11)

The vehicle state is expressed as:
- **q**: Generalized coordinates (positions, orientations)
- **q_dot**: Generalized velocities
- **q_ddot**: Generalized accelerations

### Equations of Motion Form

```
M(q) * q_ddot = Q(q, q_dot, u)
```

where:
- M(q) = Mass matrix (depends on configuration)
- Q(q, q_dot, u) = Generalized forces (functions of state and inputs)
- u = Control inputs (throttle, brake, steer)

### Solver Strategy

1. **Assemble M and Q** from vehicle state
2. **Solve linear system**: M * q_ddot = Q
3. **Integrate** using numerical integrator (Euler, RK4)
4. **Update state** for next timestep

---

## Load Transfers (Section 3.7)

Critical for understanding vehicle dynamics.

### Longitudinal Load Transfer (Section 3.7.1)

```
ΔFz = (m * ax * h) / l
```

where:
- m = vehicle mass (kg)
- ax = longitudinal acceleration (m/s²)
- h = CG height (m)
- l = wheelbase (m)

**Physical Meaning:**
- Braking → weight transfers forward
- Acceleration → weight transfers rearward
- Higher CG → more load transfer

### Lateral Load Transfer (Section 3.7.2)

**Front Axle:**
```
ΔFz_front = (m * ay * h * b/l) / tf
```

**Rear Axle:**
```
ΔFz_rear = (m * ay * h * a/l) / tr
```

where:
- ay = lateral acceleration (m/s²)
- a = CG to front axle (m)
- b = CG to rear axle (m)
- tf, tr = front, rear track widths (m)

**Distribution:**
- Roll stiffness distribution controls lateral load transfer split
- More front roll stiffness → more front lateral load transfer → understeer
- More rear roll stiffness → more rear lateral load transfer → oversteer

### Vertical Loads (Section 3.7.3)

Total load on each tire combines:
1. Static weight distribution
2. Longitudinal load transfer
3. Lateral load transfer (left/right)

---

## Sprung/Unsprung Mass System (Section 3.10)

Guiggiani separates:
- **Sprung mass**: Chassis/body (supported by springs)
- **Unsprung mass**: Wheels, brakes, uprights (below springs)

### Why Important?

1. **Ride comfort**: Unsprung mass affects road holding
2. **Load transfer dynamics**: Spring/damper rates matter
3. **High-frequency response**: Wheel hop, harshness

### Equations

**Sprung mass** (simplified vertical):
```
ms * zs_ddot = -ks*(zs - zus) - cs*(zs_dot - zus_dot) + Fother
```

**Unsprung mass**:
```
mus * zus_ddot = ks*(zs - zus) + cs*(zs_dot - zus_dot) - Ftire
```

where:
- ms, mus = sprung, unsprung masses
- zs, zus = sprung, unsprung displacements
- ks, cs = spring stiffness, damping

---

## Numerical Integration

See `core/integrator/` for integration schemes:

### Semi-Implicit Euler (Recommended for mechanics)
```
v(t+dt) = v(t) + a(t) * dt
x(t+dt) = x(t) + v(t+dt) * dt
```
Stable for mechanical systems with position-dependent forces.

### Runge-Kutta 4 (Higher accuracy)
```
k1 = f(t, x)
k2 = f(t + dt/2, x + dt*k1/2)
k3 = f(t + dt/2, x + dt*k2/2)
k4 = f(t + dt, x + dt*k3)
x(t+dt) = x(t) + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
```
More accurate but more expensive.

### Quaternion Integration (Special care needed)

For orientation quaternions:
```
q(t+dt) = q(t) + dt * 0.5 * omega ⊗ q(t)
q(t+dt) = q(t+dt) / ||q(t+dt)||  // Normalize!
```

Always normalize quaternions after integration to prevent drift.

---

## Implementation Notes

### Coordinate Systems

**World Frame (Inertial):**
- Fixed to ground
- Gravity in -Z direction
- Suitable for global motion

**Body Frame (Vehicle-Fixed):**
- Origin at CG
- X: forward, Y: left, Z: up
- Equations of motion naturally expressed here
- Guiggiani uses this extensively

**Transformations:**
Use `core/math/frames.h` for coordinate transformations.

### Force Accumulation

Forces and moments are accumulated on the sprung mass:
1. Clear force/moment accumulators
2. Add tire forces (4 wheels)
3. Add suspension forces
4. Add aerodynamic forces
5. Add weight (gravity)
6. Solve equilibrium equations

### Validation Checks

Always validate:
- Energy conservation (no artificial damping/excitation)
- Momentum conservation (when appropriate)
- Reasonable accelerations (< 30 m/s² lateral, < 15 m/s² longitudinal)
- Numerical stability (no NaN, no explosion)

---

## Usage Example

```c
#include "core/physics/dynamics_solver.h"
#include "core/physics/equations_of_motion.h"
#include "vehicle/vehicle_model.h"

// Create solver
DynamicsSolver* solver = dynamics_solver_create();

// Create vehicle
Vehicle* vehicle = vehicle_model_create();
vehicle_model_init_from_params(vehicle, "vehicle.txt");

// Simulation loop
vde_real dt = 0.001;  // 1 ms timestep
for (int i = 0; i < 10000; i++) {
    // Set inputs
    vehicle_set_inputs(vehicle, throttle, brake, steer);
    
    // Solve dynamics (three-equation model)
    dynamics_solver_solve(solver, vehicle, dt);
    
    // Log telemetry
    if (i % 100 == 0) {
        VehicleState state;
        vehicle_model_get_state(vehicle, &state);
        printf("t=%.3f: x=%.2f, y=%.2f, v=%.2f m/s\n",
               state.time, state.position.x, state.position.y,
               vde_vec3_norm(&state.velocity));
    }
}
```

---

## References

### Primary Sections:
- **Chapter 3, Sections 3.4-3.6**: Equilibrium equations
- **Chapter 3, Section 3.7**: Load transfers
- **Chapter 3, Section 3.10**: Sprung/unsprung masses
- **Chapter 3, Section 3.11**: Complete vehicle model
- **Chapter 9, Section 9.6**: Full 3D dynamics

### Related:
- Chapter 4: Braking analysis (application of equilibrium)
- Chapter 6-7: Handling (steady-state equilibrium)
- Chapter 8: Ride comfort (vertical dynamics)

---

## Implementation Status

✅ **API defined** - All interfaces specified  
📝 **Implementation needed** - See `copilotPlan.md` Phase 1, 3, 8

Priority: HIGH - Required for all vehicle simulation
