# Vehicle Components - Following Guiggiani's Methodology

This directory contains all vehicle component interfaces following the structure
presented in **"The Science of Vehicle Dynamics"** by Massimo Guiggiani.

## Guiggiani's Three-Equation Structure (Section 3.12)

All vehicle dynamics simulation follows this framework:

### 1. **CONGRUENCE (Kinematic) Equations** → `vehicle_congruence.h`
How vehicle parts move relative to each other:
- Vehicle velocities (u, v, r)
- Wheel contact point velocities
- Tire slip calculations (σ, α, φ)
- Suspension deflection rates

**Guiggiani Refs:** Chapter 3, Section 3.2; Chapter 5

### 2. **CONSTITUTIVE Equations** → `vehicle_constitutive.h`
Component behavior laws (forces from deformations):
- Tire forces from slips (Chapter 2, 10)
- Spring forces from deflections
- Damper forces from velocities
- Aerodynamic forces
- Brake torques

**Guiggiani Refs:** Chapter 3, Section 3.3; Chapter 2 (tires)

### 3. **EQUILIBRIUM Equations** → `vehicle_equilibrium.h`
Force and moment balance on vehicle:
- F = m*a (linear momentum)
- M = I*α (angular momentum)
- Load transfers
- Sprung/unsprung mass system

**Guiggiani Refs:** Chapter 3, Sections 3.4-3.6, 3.11; Chapter 9

---

## Integrated Vehicle Model → `vehicle_model.h`

Master header that ties all three equation sets together. Use this for:
- Complete vehicle simulation
- Three-equation model execution
- Handling analysis tools
- Load transfer computation

**Guiggiani Refs:** Chapter 3, Sections 3.11-3.12

---

## Individual Component Headers

### Core Vehicle Components

#### `vehicle.h` - Main Vehicle Interface
- Complete vehicle with all subsystems
- Component access methods
- High-level simulation interface

#### `sprung_mass.h` - Chassis (Body)
- Rigid body representing vehicle chassis
- 6DOF state (position, orientation, velocities)
- Mass and inertia properties
- **Guiggiani:** Chapter 3, Section 3.10; Chapter 9

#### `unsprung_mass.h` - Wheel Assemblies
- Wheel, brake, upright assembly
- One per corner (4 total)
- Vertical dynamics
- **Guiggiani:** Chapter 3, Section 3.10

---

### Tire System (Guiggiani introduces TIRES FIRST)

#### `tire.h` - Tire Interface
- Critical component generating grip forces
- Constitutive element: Forces = f(slips, load)
- **Guiggiani:** Chapter 2 (complete chapter); Chapter 10 (advanced)

See also `tire_models/` directory for:
- `magic_formula.h` - Empirical model (Section 2.10)
- `brush_models.h` - Physical model (Chapter 10)
- `tire_utilities.h` - Slip calculations (Section 2.7)

#### `wheel.h` - Wheel Dynamics
- Wheel angular velocity (spin)
- Computes tire slips from kinematics
- **Guiggiani:** Chapter 2, Section 2.11; Chapter 3, Section 3.2.7

---

### Suspension System

#### `suspension.h` - Suspension Forces
- Spring and damper constitutive laws
- Roll and vertical stiffnesses
- Load transfer computation
- **Guiggiani:** Chapter 3, Section 3.8; Chapter 8 (ride comfort)

---

### Control Systems

#### `steering.h` - Steering Geometry
- Ackermann geometry for kinematic correctness
- Steering ratio and compliance
- **Guiggiani:** Chapter 3, Section 3.2.3; Chapter 6, Section 6.18

#### `brakes.h` - Brake System
- Brake torque computation
- Optimal brake balance
- **Guiggiani:** Chapter 4 (complete chapter on braking)

#### `driveline.h` - Engine and Differential
- Engine torque transmission
- Open differential (road cars) - Chapter 6, Section 6.1
- Locked differential (race cars) - Chapter 7, Section 7.1
- **Guiggiani:** Chapter 3, Section 3.11.4

---

### Aerodynamics

#### `aerodynamics.h` - Aero Forces
- Drag, downforce, side force
- Critical for race cars (Formula cars)
- **Guiggiani:** Chapter 7, Section 7.6

---

### Parameter Management

#### `vehicle_parameters.h` - Parameter Loading
- Load vehicle parameters from files
- Guiggiani-style parameter sets
- Validation and defaults

---

## Usage Examples

### Execute Three-Equation Model for One Timestep

```c
#include "vehicle/vehicle_model.h"

Vehicle* vehicle = vehicle_model_create();
vehicle_model_init_from_params(vehicle, "data/vehicles/TBReCar.txt");

// One simulation step
vehicle_model_step(vehicle, dt);

// Or execute step-by-step:
vehicle_model_compute_congruence(vehicle);   // Step 1: Kinematics
vehicle_model_compute_constitutive(vehicle); // Step 2: Component forces
vde_real accelerations[6];
vehicle_model_assemble_equilibrium(vehicle, accelerations); // Step 3: Solve
```

### Analyze Load Transfers (Section 3.7)

```c
#include "vehicle/vehicle_equilibrium.h"

LoadTransfers transfers;
vehicle_equilibrium_compute_load_transfers(vehicle, &forces, &transfers);

printf("Longitudinal load transfer: %.2f N\n", transfers.longitudinal);
printf("Front lateral load transfer: %.2f N\n", transfers.lateral_front);
printf("Rear lateral load transfer: %.2f N\n", transfers.lateral_rear);
```

### Compute Tire Slips (Section 3.2.7)

```c
#include "vehicle/vehicle_congruence.h"

TireSlips slips[4];
vehicle_congruence_compute_all_tire_slips(vehicle, slips);

for (int i = 0; i < 4; i++) {
    printf("Wheel %d: sigma=%.3f, alpha=%.3f rad\n", 
           i, slips[i].sigma, slips[i].alpha);
}
```

---

## Implementation Status

✅ **Headers complete** - All APIs defined with Guiggiani references  
🚧 **Implementation in progress** - Following copilotPlan.md phases  

See `copilotPlan.md` for detailed implementation roadmap following
Guiggiani's pedagogical order (Tires → Vehicle → Applications).

---

## Key References

- **Chapter 2**: Tire mechanics (start here!)
- **Chapter 3**: Complete vehicle model (the core)
- **Chapter 4**: Braking performance
- **Chapter 5**: Kinematics of cornering
- **Chapter 6**: Handling of road cars
- **Chapter 7**: Handling of race cars
- **Chapter 8**: Ride comfort and road holding
- **Chapter 9**: Full 3D dynamics with roll
- **Chapter 10**: Advanced tire models

**Always refer to Guiggiani's book for mathematical derivations and physical insight.**
