# AI Assistant Context - FullCarSim

**🤖 FOR AI ASSISTANTS: Read this FIRST before making any code changes**

**Last Updated:** February 13, 2026  
**Codebase Status:** All header files defined with complete API interfaces and comprehensive Guiggiani references

---

## ⚡ Quick Start (30-Second Overview)

This is a **vehicle dynamics engine** written in **C11** for racing simulation, built as a **DLL for Unity integration**, following **Massimo Guiggiani's "The Science of Vehicle Dynamics"** methodology.

**Key Facts:**
- Language: C11 (not C++)
- Build: Native DLL for Unity
- Precision: Configurable via `vde_real` (defaults to `double`)
- Structure: Unity API boundary + Internal physics implementation
- Math: Complete 3D math library in `include/core/math/` 
- Architecture: Guiggiani's **three-equation structure** (Congruence → Constitutive → Equilibrium)
- Status: Headers complete with Guiggiani references, implementations in progress

**Before coding anything:**
1. Read the [Two-Level Architecture](#two-level-architecture-unity--physics) below
2. Check [Standards Summary](#standards-summary)
3. Understand [Guiggiani's Three-Equation Structure](#guiggianis-three-equation-structure)
4. Review [What's Implemented](#whats-implemented)

---

## 🎮 Two-Level Architecture: Unity ↔ Physics

**CRITICAL:** This codebase has two distinct layers with different concerns:

### Layer 1: Unity DLL Interface (unity_api.h)
**Purpose:** Simple, stable API for Unity C# integration

**Responsibilities:**
- Export C functions for Unity to call
- Simple data structures (doubles, arrays) for easy C# marshaling
- Render data (positions, orientations, wheel states)
- Telemetry output (comprehensive datasets)
- Asset loading (vehicle configs, track maps)
- Error handling and validation

**Key principle:** Unity shouldn't know about Guiggiani, tire slips, or three-equation structure. It just:
1. Sends inputs (throttle, brake, steering)
2. Steps physics
3. Gets render data (positions, orientations)
4. Displays result

→ See **[UNITY_INTEGRATION.md](UNITY_INTEGRATION.md)** for complete guide

### Layer 2: Internal Physics (Guiggiani Implementation)
**Purpose:** High-fidelity vehicle dynamics simulation

**Responsibilities:**
- Three-equation model (Congruence → Constitutive → Equilibrium)
- Tire models (Magic Formula, Brush)
- Load transfers, suspension dynamics
- 6DOF rigid body dynamics
- Numerical integration

**Key principle:** Follow Guiggiani's book rigorously for physical correctness.

→ See sections below for Guiggiani architecture

---

## 🏗️ Guiggiani's Three-Equation Structure

**The core architecture principle of this codebase** (Guiggiani Section 3.12):

### 1. 🔵 CONGRUENCE (Kinematic) Equations
**Header:** `vehicle/vehicle_congruence.h`  
**Purpose:** How vehicle parts move relative to each other

- Vehicle velocities (u, v, w, p, q, r)
- Wheel contact point velocities
- **Tire slip calculations** (σ, α, φ)
- Suspension deflection rates

### 2. 🟢 CONSTITUTIVE Equations
**Header:** `vehicle/vehicle_constitutive.h`  
**Purpose:** Component behavior (forces from deformations/velocities)

- Tire forces from slips (see `tire_models/`)
- Spring forces from deflections
- Damper forces from velocities
- Aerodynamic forces from velocity
- Brake torques from pressure

### 3. 🔴 EQUILIBRIUM Equations
**Header:** `vehicle/vehicle_equilibrium.h`  
**Purpose:** Force and moment balance

- F = m·a (linear momentum)
- M = I·α (angular momentum)
- Load transfers
- Sprung/unsprung mass system

### Master Integration
**Header:** `vehicle/vehicle_model.h`  
Orchestrates all three equation types in proper sequence

---

## 📋 Standards Summary

### Critical Rules (Never Break These)

1. **Use `#pragma once`** - Never `#ifndef` guards
2. **Use `vde_real`** - Never bare `double` or `float`
3. **Use math types** - `vde_vec3`, `vde_quat`, `vde_mat3`, `vde_frame`
4. **Defensive programming** - Always check nulls, validate inputs
5. **Section headers** - Organize code with `//-------------------------`
6. **Include core/math/math_base.h** - Always include for `vde_real` and constants
7. **Use correct include paths** - Math library is in `"core/math/"`, not `"math/"`

### Quick Style Reference

```c
#pragma once
// Vehicle Dynamics Engine - Module Name
core/
#include "math/math_base.h"

//-------------------------
// Types
//-------------------------

typedef struct Module {
    vde_real value;
} Module;

//-------------------------
// API Functions
//-------------------------

VDE_API Module* module_create(void);
VDE_API void module_destroy(Module* m);
VDE_API int module_update(Module* m, vde_real dt);
```

---

## 🗺️ Project Structure

```
FullCarSim/
├── include/                    # Public headers (ALL with Guiggiani references)
│   │
│   ├── unity_api.h            # ⭐ UNITY DLL INTERFACE (Layer 1)
│   │                          #    - Export boundary for Unity C#
│   │                          #    - VehicleRenderData (positions, wheels)
│   │                          #    - TelemetryFrame (datasets)
│   │                          #    - DriverInputs (controls)
│   │                          #    - Asset loading (configs, maps)
│   │
│   ├── core/                   # Core engine subsystems (Layer 2)
│   │   ├── math/              # ✅ COMPLETE - 3D math library
│   │   ├── integrator/        # ✅ API DEFINED - Numerical integrators
│   │   ├── physics/           # ✅ API DEFINED - Physics solvers
│   │   └── utils/             # ✅ API DEFINED - Utilities (logger, etc.)
│   │
│   ├── vehicle/               # ✅ API DEFINED - Vehicle components (Layer 2)
│   │   ├── vehicle_model.h        # ⭐ Master header - three-equation structure
│   │   ├── vehicle_congruence.h   # 🔵 CONGRUENCE equations (NEW)
│   │   ├── vehicle_constitutive.h # 🟢 CONSTITUTIVE equations (NEW)
│   │   ├── vehicle_equilibrium.h  # 🔴 EQUILIBRIUM equations (NEW)
│   │   ├── vehicle.h              # Main vehicle interface
│   │   ├── sprung_mass.h          # Chassis rigid body (Guiggiani 3.10, Ch 9)
│   │   ├── unsprung_mass.h        # Wheel assemblies (Guiggiani 3.10)
│   │   ├── tire.h                 # Tire interface (Guiggiani Ch 2)
│   │   ├── wheel.h                # Wheel dynamics (Guiggiani 2.11, 3.2.7)
│   │   ├── suspension.h           # Suspension forces (Guiggiani 3.8, Ch 8)
│   │   ├── steering.h             # Steering geometry (Guiggiani 3.2.3)
│   │   ├── brakes.h               # Brake system (Guiggiani Ch 4)
│   │   ├── driveline.h            # Engine/differential (Guiggiani 3.11.4, 6.1, 7.1)
│   │   ├── aerodynamics.h         # Aero forces (Guiggiani 7.6)
│   │   ├── vehicle_parameters.h   # Parameter management
│   │   └── README.md              # 📄 Vehicle components guide
│   │
│   ├── tire_models/           # ✅ API DEFINED - Tire models (Guiggiani Ch 2, 10)
│   │   ├── magic_formula.h        # Pacejka model (Guiggiani 2.10)
│   │   ├── brush_models.h         # Brush model (Guiggiani Ch 10)
│   │   ├── tire_utilities.h       # Slip calculations (Guiggiani 2.7)
│   │   └── README.md              # 📄 Tire models guide
│   │
│   ├── simulation/            # ✅ API DEFINED - Simulation control
│   ├── input/                 # ✅ API DEFINED - Control inputs
│   ├── track/                 # ✅ API DEFINED - Track/environment
│   └── README.md              # 📄 Include directory overview
│
├── src/                       # Implementations (mirrors include/)
│   ├── core/                  # Core engine implementations
│   │   ├── math/             # ✅ COMPLETE - Math implementations
│   │   ├── integrator/       # 📝 TODO - Needs implementation
│   │   ├── physics/          # 📝 TODO - Needs implementation
│   │   └── utils/            # 📝 TODO - Needs implementation
│   ├── vehicle/              # 🚧 PARTIAL - Basic 2D vehicle.c exists
│   ├── tire_models/          # 📝 TODO - Needs implementation
│   ├── simulation/           # 🚧 PARTIAL - Basic sim loop working
│   ├── input/                # 📝 TODO - Needs implementation
│   ├── track/                # 📝 TODO - Needs implementation
│   └── unity_api.c           # 📝 TODO - Unity API implementation
│
├── unityInterface/           # Unity-specific build and test
│   ├── include/sim_api.h    # Legacy API (migrate to unity_api.h)
│   └── src/                 # Build helpers
│
├── data/                     # Runtime assets (loaded by Unity)
│   ├── vehicles/            # Vehicle configs (TBReCar.txt, etc.)
│   ├── tracks/              # Track definitions (skidpad.txt, etc.)
│   └── configs/             # Simulation configs
│
├── build/                    # Build outputs (DLL here)
│
├── AI_CONTEXT.md             # 📄 This file - AI assistant guide
├── UNITY_INTEGRATION.md      # 📄 Unity DLL integration guide ⭐ NEW
├── CODING_STANDARDS.md       # 📄 Coding standards - READ FIRST
├── copilotPlan.md            # 📄 Implementation roadmap (Guiggiani order)
└── README.md                 # 📄 Project overview
```

**Key Insight:** Notice the clear separation between Unity API (unity_api.h) and
internal physics implementation (vehicle/, tire_models/, core/physics/).

**Data Flow:**
```
Unity C# → unity_api.h → vehicle_model.h → Guiggiani physics → unity_api.h → Unity C#
         (inputs)        (step)           (simulation)        (render data)
```

---

## ✅ What's Implemented

### Math Library (`include/core/math/`, `src/core/math/`) - ✅ COMPLETE

**Status:** Production-ready, follows all standards

| File | Purpose | Key Features |
|------|---------|--------------|
| `math_base.h/c` | Core types, constants | `vde_real`, `VDE_PI`, helper functions |
| `vec3.h/c` | 3D vectors | Add, sub, dot, cross, normalize |
| `mat3.h/c` | 3x3 matrices | Multiply, determinant, inverse, transforms |
| `quat.h/c` | Quaternions | SLERP, rotation, conversion, integration |
| `frames.h/c` | Rigid transforms | Body↔world transforms, composition |

**Key types:**
```c
vde_real           // Configurable precision (double by default)
vde_vec3           // {x, y, z}
vde_mat3           // 3x3 matrix, column-major
vde_quat           // {w, x, y, z}
vde_frame          // {quat q, vec3 p} - rigid body pose
```

**Key constants:**
```c
VDE_PI             // π
VDE_DEG2RAD        // Degrees to radians
VDE_RAD2DEG        // Radians to degrees
VDE_EPSILON        // Small number for comparisons
```

---

### Three-Equation Structure Headers - ✅ API COMPLETE

**NEW ADDITION:** Headers that make Guiggiani's methodology explicit

| Header | Purpose | Guiggiani Ref |
|--------|---------|---------------|
| `vehicle_model.h` | Master integration header | Ch 3, Sec 3.11-3.12 |
| `vehicle_congruence.h` | Kinematic equations | Ch 3, Sec 3.2 |
| `vehicle_constitutive.h` | Component behavior | Ch 3, Sec 3.3 |
| `vehicle_equilibrium.h` | Force/moment balance | Ch 3, Sec 3.4-3.6, 3.11 |

**These headers provide:**
- Clear separation of equation types
- Direct mapping to Guiggiani's book
- Comprehensive API for each phase
- Integration with existing component headers

---

### Vehicle Components (`include/vehicle/`) - ✅ API DEFINED

**Status:** All component APIs fully defined with comprehensive Guiggiani references

**Complete headers with Guiggiani annotations:**
- `vehicle.h` - Main vehicle interface
- `tire.h` - Tire component (Guiggiani Ch 2)
- `wheel.h` - Wheel dynamics (Guiggiani 2.11, 3.2.7)
- `suspension.h` - Suspension forces (Guiggiani 3.8, Ch 8)
- `unsprung_mass.h` - Unsprung mass dynamics (Guiggiani 3.10)
- `sprung_mass.h` - Sprung mass (chassis) rigid body (Guiggiani 3.10, Ch 9)
- `brakes.h` - Brake system (Guiggiani Ch 4)
- `steering.h` - Steering system (Guiggiani 3.2.3, 6.18)
- `driveline.h` - Driveline (Guiggiani 3.11.4, 6.1, 7.1)
- `aerodynamics.h` - Aerodynamic forces (Guiggiani 7.6)
- `vehicle_parameters.h` - Vehicle parameter management

**Implementation status:** Basic 2D vehicle.c exists, needs expansion

---

### Tire Models (`include/tire_models/`) - ✅ API DEFINED

**Status:** APIs defined with extensive Guiggiani references

- `magic_formula.h` - Pacejka Magic Formula (Guiggiani 2.10)
- `brush_models.h` - Brush tire model (Guiggiani Ch 10)
- `tire_utilities.h` - Tire utility functions (Guiggiani 2.7, 3.2.7)

**Implementation status:** TODO - Phase 2 priority (START WITH MAGIC FORMULA)

---

### Core Physics (`include/core/physics/`) - ✅ API DEFINED

- `equations_of_motion.h` - 6DOF equations (Guiggiani 3.4-3.6, Ch 9)
- `dynamics_solver.h` - Main dynamics solver (Guiggiani 3.11)
- `constraints.h` - Constraint solver

**Implementation status:** TODO - Phase 1, 3 priority

---

### Simulation System (`include/simulation/`) - ✅ WORKING

- `simulation_config.h` - Main simulation API ✅ IMPLEMENTED
- `simulation_loop.h` - Simulation loop structure (API defined)
- `telemetry.h` - Telemetry system (API defined)

**Current capabilities:**
- Configurable timestep
- Telemetry callbacks
- State export
- Input validation

**Implementation status:** Basic loop working in `src/simulation/sim.c`

---

### Other Subsystems - ✅ API DEFINED

All have complete APIs with Guiggiani references:
- **Track** (`include/track/`) - Surface, geometry, friction (Guiggiani Ch 5)
- **Input** (`include/input/`) - Control input system
- **Integrators** (`include/core/integrator/`) - Semi-implicit Euler, RK4
- **Utils** (`include/core/utils/`) - Logger

**Implementation status:** TODO - See copilotPlan.md for priorities

---

### Documentation - ✅ COMPREHENSIVE

**NEW ADDITION:** Comprehensive README files

- `include/README.md` - Overview of entire include directory
- `include/vehicle/README.md` - Vehicle components guide
- `include/tire_models/README.md` - Tire models guide
- `include/core/physics/README.md` - Physics solvers guide

Each provides:
- Organization explanation
- Guiggiani references
- Usage examples
- Integration with three-equation structure

---

### Summary Status

- ✅ **core/math/** - Complete (headers + implementations)
- ✅ **All subsystem headers** - Complete API definitions with Guiggiani refs
- ✅ **Three-equation structure headers** - NEW, complete
- ✅ **Comprehensive documentation** - README files with examples
- 🚧 **simulation/sim.c** - Basic 2D implementation working
- 📝 **All other source files** - Placeholder implementations, APIs defined

**Next Priority:** Implement Magic Formula tire model (copilotPlan.md Phase 2)

---

## 📝 Common Tasks

### Understanding the Three-Equation Structure

**Before implementing ANY vehicle dynamics code, understand this:**

```c
// Guiggiani's three-equation model (Section 3.12)

// 1. CONGRUENCE (vehicle_congruence.h)
//    Input: vehicle state (position, velocity, orientation)
//    Output: tire slips, suspension velocities, etc.
TireSlips slips[4];
vehicle_congruence_compute_all_tire_slips(vehicle, slips);

// 2. CONSTITUTIVE (vehicle_constitutive.h + tire_models/)
//    Input: tire slips, suspension deflections
//    Output: forces and moments
TireForces forces[4];
for (int i = 0; i < 4; i++) {
    vehicle_constitutive_evaluate_tire(vehicle, i, &slips[i], &forces[i]);
}

// 3. EQUILIBRIUM (vehicle_equilibrium.h)
//    Input: all forces and moments
//    Output: accelerations
vde_vec3 linear_accel, angular_accel;
VehicleForces all_forces;
vehicle_equilibrium_assemble_forces(vehicle, &all_forces);
EquationsOfMotion eom;
vehicle_equilibrium_build_equations(vehicle, &all_forces, &eom);
vehicle_equilibrium_solve_accelerations(&eom, &linear_accel, &angular_accel);

// 4. INTEGRATE (core/integrator/)
//    Input: accelerations
//    Output: new state
integrator_step(vehicle, &linear_accel, &angular_accel, dt);
```

**Or use the master header:**
```c
#include "vehicle/vehicle_model.h"

// All three equations executed in one call
vehicle_model_step(vehicle, dt);
```

---

### Adding a New Module

1. Create header in `include/module_name/`
   ```c
   #pragma once
   // Vehicle Dynamics Engine - Module Name
   
   #include "math/math_base.h"
   
   //-------------------------
   // Types
   //-------------------------
   
   typedef struct ModuleName ModuleName;
   
   //-------------------------
   // API Functions
   //-------------------------
   
   VDE_API ModuleName* module_create(void);
   VDE_API void module_destroy(ModuleName* m);
   ```

2. Create implementation in `src/module_name/`
   ```c
   #include "module_name/module_name.h"
   
   //-------------------------
   // Internal State
   //-------------------------
   
   struct ModuleName {
       vde_real data;
   };
   
   //-------------------------
   // Lifecycle
   //-------------------------
   
   ModuleName* module_create(void) {
       ModuleName* m = malloc(sizeof(ModuleName));
       if (!m) return NULL;
       m->data = (vde_real)0.0;
       return m;
   }
   
   void module_destroy(ModuleName* m) {
       if (!m) return;
       free(m);
   }
   ```

3. Add to build system (CMakeLists.txt)

### Using Math Library

```c
// Vectors
vde_vec3 pos = vde_vec3_make(1.0, 2.0, 3.0);
vde_vec3 vel = vde_vec3_zero();
vde_vec3_add(&result, &pos, &vel);
vde_real len = vde_vec3_norm(&pos);

// Quaternions
vde_quat q = vde_quat_identity();
vde_vec3 axis = vde_vec3_make(0, 0, 1);
vde_quat rot = vde_quat_from_axis_angle(&axis, VDE_PI / 4.0);
vde_vec3 rotated = vde_quat_rotate(&rot, &pos);

// Frames (rigid transforms)
vde_frame body = vde_frame_identity();
vde_vec3 world_pos = vde_frame_point_body_to_world(&body, &pos);
```

### Adding Defensive Checks

```c
void module_function(Module* m, vde_real value) {
    // Check null pointer
    if (!m) return;
    
    // Validate input
    if (value < (vde_real)0.0) return;
    
    // Clamp if needed
    value = vde_clamp(value, (vde_real)0.0, (vde_real)1.0);
    
    // Check for numerical issues
    m->state += value;
    if (!vde_isfinite(m->state)) {
        m->state = (vde_real)0.0;  // Reset to safe value
    }
}
```

---

## 🚨 Common Mistakes to Avoid

### ❌ DON'T DO THIS:

```c
// Wrong: Using double instead of vde_real
double position = 10.0;

// Wrong: Traditional header guards
#ifndef MODULE_H
#define MODULE_H

// Wrong: No null check
void update(Module* m) {
    m->value = 5.0;  // Crash if m is NULL!
}

// Wrong: Creating custom vector types
struct MyVector { double x, y, z; };

// Wrong: No input validation
5. **Math API reference:** Look at `include/core/math/*.h` headers
    v->speed = speed;  // Could be negative or NaN!
}
```

### ✅ DO THIS INSTEAD:

```c
// Correct: Using vde_real
vde_real position = (vde_real)10.0;
core/math/** - Complete, production-ready
- ✅ **simulation/** - Basic loop working, structure solid
- 🚧 **vehicle/** - Basic 2D implementation, component structure created
- 📝 **core/integrator/** - Structure created, needs implementation
- 📝 **core/physics/** - Structure created, needs implementation
- 📝 **core/utils/** - Structure created, needs implementation
- 📝 **tire_models/** - Structure created, needs implementation
- 📝 **track/** - Structure created, needs implementation
- 📝 **input/** - Structure created, needs implementation
// Correct: Null check
void update(Module* m) {
    if (!m) return;
    m->value = (vde_real)5.0;
}

// Correct: Use math library types
vde_vec3 position = vde_vec3_make(1, 2, 3);

// Correct: Validate and check
void set_speed(Vehicle* v, vde_real speed) {
    if (!v) return;
    speed = vde_clamp(speed, (vde_real)0.0, (vde_real)100.0);
    if (!vde_isfinite(speed)) return;
    v->speed = speed;
}
```

---

## 🔍 When You Need More Details

**Unity Integration:**
1. **Unity DLL API:** See `include/unity_api.h` (THE Unity boundary)
2. **Integration guide:** See `UNITY_INTEGRATION.md` ⭐ ESSENTIAL for Unity work
3. **Asset management:** See `UNITY_INTEGRATION.md` - Asset Management section
4. **Telemetry system:** See `UNITY_INTEGRATION.md` - Telemetry section

**Internal Physics:**
5. **Project organization:** See `include/README.md`
6. **Vehicle components:** See `include/vehicle/README.md`
7. **Tire models:** See `include/tire_models/README.md` (extensive guide)
8. **Physics solvers:** See `include/core/physics/README.md`
9. **Detailed standards:** See `CODING_STANDARDS.md`
10. **Project roadmap:** See `copilotPlan.md` (follows Guiggiani order)
11. **Build instructions:** See `README.md`
12. **Math API reference:** Look at `include/core/math/*.h` headers
13. **Component APIs:** All headers in `include/` have complete function APIs
14. **Working examples:** Check `src/vehicle/vehicle.c`, `src/simulation/sim.c`
15. **Guiggiani references:** Every header has chapter/section annotations

---

## 🔄 Memory Aid: Analysis Shortcuts

**Instead of analyzing the whole codebase each time, remember:**

### Core Architecture (MEMORIZE THESE):
1. **Two-layer design**: Unity API (simple) ↔ Internal Physics (complex)
2. **Unity boundary**: `unity_api.h` - THE DLL export interface
3. **Three-equation structure**: Congruence → Constitutive → Equilibrium (Guiggiani 3.12)
4. **Data flow**: Unity inputs → Physics sim → Unity render data
5. **Asset management**: Unity loads files → DLL parses → Internal storage

### Core Principles:
1. **Precision abstraction**: `vde_real`
2. **Math library exists**: `vec3`, `quat`, `mat3`, `frame`
3. **Defensive**: Check nulls, validate inputs, check finite
4. **Organized**: Section headers, clear structure
5. **Modern C**: `#pragma once`, `<stdbool.h>`, `restrict`
6. **Guiggiani-first**: All headers reference the book

### Quick Type Lookup:
- Scalar math → `vde_real`
- 3D position/velocity → `vde_vec3`
- Rotation → `vde_quat`
- Transform → `vde_frame`
- Matrix → `vde_mat3`

### Unity API Data Structures:
- Render data → `VehicleRenderData` (positions, wheels, visual feedback)
- Driver inputs → `DriverInputs` (throttle, brake, steering, gear)
- Telemetry → `TelemetryFrame` (73+ channels for datasets)
- Vehicle config → `VehicleParameters` (mass, inertia, etc.)
- Track info → `TrackInfo`, `SurfaceQuery`

### Three-Equation Headers (Internal Physics):
- Congruence (kinematics) → `vehicle/vehicle_congruence.h`
- Constitutive (component behavior) → `vehicle/vehicle_constitutive.h`
- Equilibrium (force balance) → `vehicle/vehicle_equilibrium.h`
- Master integration → `vehicle/vehicle_model.h`

### Module Status at a Glance:
- ✅ **unity_api.h** - Complete Unity interface definition (NEW)
- ✅ **core/math/** - Complete (headers + implementations)
- ✅ **All headers** - Complete API definitions with Guiggiani refs
- ✅ **Three-equation structure** - NEW headers created
- ✅ **Documentation** - Comprehensive README files + UNITY_INTEGRATION.md
- 🚧 **simulation/sim.c** - Basic 2D implementation working
- 📝 **unity_api.c** - TODO: Implement Unity API layer
- 📝 **Most source files** - Placeholder implementations, APIs defined

### Documentation Quick Reference:
- `UNITY_INTEGRATION.md` - ⭐ Complete Unity DLL guide (NEW)
- `unity_api.h` - ⭐ THE DLL boundary (NEW)
- `include/README.md` - Overview of entire structure
- `include/vehicle/README.md` - Vehicle components guide
- `include/tire_models/README.md` - Detailed tire model guide
- `include/core/physics/README.md` - Physics solvers guide
- `copilotPlan.md` - Implementation roadmap following Guiggiani

---

## 📌 Most Important Reminders

**Architecture:**
1. **TWO LAYERS:** Unity API (simple, stable) vs. Internal Physics (complex, accurate)
2. **Unity boundary:** Keep unity_api.h simple - no Guiggiani concepts exposed
3. **Data flow:** Unity sends inputs → Physics simulates → Unity gets render data

**Guiggiani Physics:**
4. **Three-equation structure:** Congruence → Constitutive → Equilibrium (Section 3.12)
5. **Follow Guiggiani:** All math and physics comes from the book
6. **Tires first:** Chapter 2 before vehicle model (Guiggiani's teaching order)

**Code Quality:**
7. **READ** `CODING_STANDARDS.md` Quick Reference section first
8. **READ** relevant README files for context
9. **USE** the math library - don't reinvent vectors/quaternions
10. **CHECK** nulls and validate inputs - always defensive
11. **REFER** to header Guiggiani annotations for chapter/section references

**Unity Integration:**
12. **Render data:** Must be complete and Unity-friendly (positions, quaternions)
13. **Telemetry:** Comprehensive for dataset generation (73+ channels)
14. **Assets:** Clear loading mechanism (vehicle configs, track maps)
15. **Performance:** Physics at 1000 Hz, rendering at 60 Hz - design accordingly

**Documentation:**
16. **UPDATE** documentation when you discover new important patterns

---

## ⚠️ Keep This Updated!

When you:
- Add new modules
- Establish new patterns
- Complete major features
- Change existing conventions
- **Modify Unity API** (breaking changes!)

**→ Update this file with a new date stamp at the top**

This prevents future AI instances from having to re-analyze everything!

---

**Ready to code?** 
→ Quick checklist: 
- Two-layer architecture understood? ✓
- Unity API vs Internal Physics separation clear? ✓
- Three-equation structure understood? ✓
- `vde_real` types? ✓ 
- Math types? ✓ 
- Null checks? ✓ 
- Sections? ✓
- Guiggiani reference checked? ✓
- Unity integration needs considered? ✓

Good luck! 🚀
