# FullCarSim Include Directory

**Comprehensive vehicle dynamics headers following Guiggiani's "The Science of Vehicle Dynamics"**

## Directory Organization

```
include/
├── core/                    Core engine subsystems
│   ├── math/               ✅ 3D math library (complete)
│   ├── integrator/         ✅ Numerical integrators (Euler, RK4, Semi-implicit)
│   ├── physics/            📝 Dynamics solvers
│   └── utils/              ✅ Utilities (logger)
│
├── vehicle/                Vehicle components (Guiggiani Ch 3)
│   ├── vehicle_model.h     ⭐ Master header - three-equation structure
│   ├── vehicle_congruence.h    🔵 CONGRUENCE equations (kinematics)
│   ├── vehicle_constitutive.h  🟢 CONSTITUTIVE equations (component behavior)
│   ├── vehicle_equilibrium.h   🔴 EQUILIBRIUM equations (force balance)
│   ├── vehicle.h           Main vehicle interface
│   ├── sprung_mass.h       Chassis rigid body
│   ├── unsprung_mass.h     Wheel assemblies
│   ├── tire.h              Tire interface
│   ├── wheel.h             Wheel dynamics
│   ├── suspension.h        Suspension forces
│   ├── steering.h          Steering geometry
│   ├── brakes.h            Brake system
│   ├── driveline.h         Engine and differential
│   ├── aerodynamics.h      Aero forces
│   └── vehicle_parameters.h Parameter management
│
├── tire_models/            Tire constitutive models (Guiggiani Ch 2, 10)
│   ├── magic_formula.h     Pacejka model (empirical)
│   ├── brush_models.h      Brush model (physical)
│   └── tire_utilities.h    Slip calculations
│
├── track/                  Track/environment
│   ├── track_surface.h     Surface properties
│   ├── track_geometry.h    Track geometry
│   └── friction_map.h      Friction mapping
│
├── simulation/             Simulation control
│   ├── simulation_config.h Main simulation API
│   ├── simulation_loop.h   Simulation loop
│   └── telemetry.h         Data recording
│
└── input/                  Control inputs
    └── control.h           Driver input system
```

---

## 🌟 Start Here: Guiggiani's Three-Equation Structure

The complete vehicle model is based on three types of equations (Section 3.12):

### 1. 🔵 CONGRUENCE (Kinematic) Equations
**Header:** `vehicle/vehicle_congruence.h`  
**Reference:** Chapter 3, Section 3.2

Describes how vehicle parts move relative to each other:
- Vehicle velocities (u, v, w, p, q, r)
- Wheel contact point velocities
- **Tire slip calculations** (σ, α, φ) - critical connection!
- Suspension deflection rates

### 2. 🟢 CONSTITUTIVE Equations
**Header:** `vehicle/vehicle_constitutive.h`  
**Reference:** Chapter 3, Section 3.3; Chapters 2, 10

Component behavior laws:
- **Tire forces** from slips: [Fx, Fy, Fz] = f(σ, α, Fz, ...)
  - See `tire_models/magic_formula.h` - empirical (Section 2.10)
  - See `tire_models/brush_models.h` - physical (Chapter 10)
- **Spring forces** from deflections: F = k * Δz
- **Damper forces** from velocities: F = c * v
- **Aerodynamic forces**: Drag, downforce (Chapter 7)
- **Brake torques**: T = f(pressure)

### 3. 🔴 EQUILIBRIUM Equations
**Header:** `vehicle/vehicle_equilibrium.h`  
**Reference:** Chapter 3, Sections 3.4-3.6, 3.11

Force and moment balance:
- **F = m·a** (linear momentum)
- **M = I·α** (angular momentum)
- Load transfers (Section 3.7)
- Sprung/unsprung mass system (Section 3.10)

### Master Integration
**Header:** `vehicle/vehicle_model.h`  
Orchestrates all three equation types in proper sequence.

---

## 📚 Guiggiani's Pedagogical Order

### Chapter 2: Start with TIRES! 🏁
**Directory:** `tire_models/`

Guiggiani introduces tires FIRST because:
1. Tires generate all grip forces
2. Highly nonlinear behavior
3. Critical for all vehicle dynamics
4. Must understand before vehicle model

**Implementation Priority:** HIGH (tire forces are the foundation of all grip)

### Chapter 3: Integrated Vehicle Model 🚗
**Directory:** `vehicle/`

Complete vehicle with ALL subsystems:
- Tires, suspensions, chassis together
- Three-equation structure
- Load transfers fundamental
- Not a "build up" approach - integrated from start

**Implementation Priority:** HIGH - full 3D vehicle is the main near-term goal

### Chapters 4-7: Applications 🏎️
- Chapter 4: Braking (`vehicle/brakes.h`)
- Chapter 5: Kinematics of cornering (`vehicle/vehicle_congruence.h`)
- Chapter 6: Road car handling (`vehicle/driveline.h` - open diff)
- Chapter 7: Race car handling (`vehicle/aerodynamics.h`, locked diff)

**Implementation Priority:** MEDIUM - after all basic handling cases

### Chapter 8: Ride Comfort 🛣️
**Directory:** `vehicle/suspension.h`

Quarter car and full vehicle ride analysis.

**Implementation Priority:** MEDIUM - after vehicle model core is solid

### Chapter 9: Full 3D Dynamics 🌐
**Directory:** `vehicle/sprung_mass.h`, `core/math/`

Roll, pitch, yaw - complete 6DOF.

**Implementation Priority:** HIGH - required for realistic load transfers and ride

### Chapter 10: Advanced Tire Theory 🔬
**Directory:** `tire_models/brush_models.h`

Physical brush model - detailed theory.

**Implementation Priority:** LATER (after Magic Formula working)

---

## 🚀 Quick Start Guide

### For AI Assistants:
1. **Read** `AI_CONTEXT.md` first - architecture and 30-second overview
2. **Check** `AI_CONTEXT.md` > Standards Summary for coding rules
3. **See** `AI_CONTEXT.md` > Next Priorities for implementation order

### For Developers:
1. Start with `vehicle/vehicle_model.h` - understand structure
2. Explore three-equation headers:
   - `vehicle_congruence.h`
   - `vehicle_constitutive.h`
   - `vehicle_equilibrium.h`
3. See component READMEs:
   - `vehicle/README.md`
   - `tire_models/README.md`
   - `core/physics/README.md`

---

## 📖 Header Documentation Style

All headers now include:
- **Guiggiani chapter/section references**
- **Key concepts** explained
- **Mathematical formulas** from the book
- **Integration** with three-equation structure
- **Implementation notes**

Example:
```c
#pragma once
// Vehicle Dynamics Engine - Tire Component
// Guiggiani Chapter 2: "Mechanics of the Wheel with Tire"
//
// The tire is the critical vehicle component...
//
// Key Sections:
//   - 2.7: Tire slips (σ, α, φ)
//   - 2.8: Grip forces vs slips
//   - 2.10: Magic Formula
```

---

## 🔧 Build System Integration

Headers are organized for CMake:
```cmake
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Components can include:
#include "vehicle/vehicle_model.h"        # Master header
#include "vehicle/tire.h"                 # Component
#include "tire_models/magic_formula.h"    # Tire model
#include "core/math/vec3.h"              # Math library
```

---

## ✅ Implementation Status

### Complete (Headers + Implementation):
- ✅ `core/math/` - Full 3D math library
- ✅ `core/integrator/` - Euler, RK4, Semi-implicit Euler
- ✅ `core/utils/logger` - Logging with levels, file output, callbacks
- ✅ `tire_models/magic_formula` - Full Pacejka Magic Formula
- ✅ `tire_models/brush_models` - Simplified brush model
- ✅ `tire_models/tire_utilities` - Slip calculations
- ✅ `vehicle/tire.c` - Tire component with model dispatch
- ✅ `simulation/simulation_config.h` - Basic simulation API

### Scaffolded (structure + placeholder logic):
- 🚧 `vehicle/vehicle_congruence.c` - Ackermann works; others return zeros
- 🚧 `vehicle/vehicle_constitutive.c` - Linear suspension works; tires return zeros
- 🚧 `vehicle/vehicle_equilibrium.c` - Load transfer works; force assembly returns zeros
- 🚧 `vehicle/vehicle_model.c` - State retrieval works; full step pending
- 🚧 `vehicle/wheel`, `suspension`, `brakes`, `aero`, `steering`, `driveline` - Partial

### API Only (headers complete, implementations pending):
- 📝 `core/physics/` - Structures defined, solver logic pending
- 📝 `track/` - APIs defined, basic structures only

### Next Steps:
See `AI_CONTEXT.md` > Next Priorities for implementation order.

---

## 🎯 Key Design Principles

Following Guiggiani and coding standards from `AI_CONTEXT.md`:

1. **Precision abstraction:** `vde_real` for all floating-point
2. **Math library:** Use `vde_vec3`, `vde_quat`, `vde_frame`
3. **Defensive programming:** Null checks, validation, finite checks
4. **Section headers:** Organize with `//-------------------------`
5. **Modern C:** `#pragma once`, `<stdbool.h>`, `restrict`
6. **Physical correctness:** Follow Guiggiani's equations exactly

---

## 📚 Essential References

**Book:** "The Science of Vehicle Dynamics: Handling, Braking, and Ride  
         of Road and Race Cars" by Massimo Guiggiani

**Key Chapters:**
- Ch 2: Tire mechanics (START HERE)
- Ch 3: Vehicle model (THE CORE)
- Ch 4-7: Applications
- Ch 8: Ride comfort
- Ch 9: 3D dynamics
- Ch 10: Advanced tires

**Project Files:**
- `AI_CONTEXT.md` - Architecture, standards, and implementation status
- `README.md` - Project overview and build guide

---

## 🤝 Contributing

When adding new headers:
1. Include Guiggiani chapter/section references
2. Explain physical concepts
3. Show formulas from the book
4. Integrate with three-equation structure
5. Update relevant README files
6. Follow coding standards from `AI_CONTEXT.md`

---

**For questions or clarification, refer to Guiggiani's book. The mathematics  
and physics are all there - we're implementing what's already rigorously  
derived!**
