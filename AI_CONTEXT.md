# AI Assistant Context - FullCarSim

**🤖 FOR AI ASSISTANTS: Read this FIRST before making any code changes**

**Last Updated:** February 19, 2026  
**Codebase Status:** Unity API fully implemented and wired up. Complete working simulation with simple 2D dynamics. All header files defined with complete API interfaces and comprehensive Guiggiani references. Integrator implementations complete. Three-equation structure source files scaffolded (Ackermann and load transfer work; tire force paths return zeros pending full 3D Vehicle expansion). Tire models fully implemented. `src/simulation/` removed (dead code) — debug harness now drives entirely through `unity_api.c` for code-path parity with the DLL. **Vehicle parameter system fully consolidated** — single `static const VehicleParameters s_tbre_defaults` in `vehicle_parameters.c` is the sole source of truth for all TBRe FSAE defaults; `vehicle_params_load_from_file()` overrides at runtime; `vehicle_params_apply_to_*()` functions propagate to components at init with zero runtime overhead.

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

### Layer 1: Unity DLL Interface (unity_api.h + unity_api.c)
**Purpose:** Simple, stable API for Unity C# integration

**Location:**
- Header: `include/unity_api.h` (~500 lines with comprehensive documentation)
- Implementation: `src/unity_api.c` (~550 lines) ✅ FULLY IMPLEMENTED
- Builds into: `racing_sim.dll`

**Responsibilities:**
- Export C functions for Unity to call (UNITY_API macro)
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

**Unity DLL API:**
```c
// Lifecycle
VehicleSimulation* VehicleSim_Create(double timestep)
void VehicleSim_Destroy(VehicleSimulation* sim)
int VehicleSim_Initialize(VehicleSimulation* sim)
void VehicleSim_Reset(VehicleSimulation* sim)

// Simulation - NEW: Step function takes inputs directly
void VehicleSim_Step(VehicleSimulation* sim, const DriverInputs* inputs)
void VehicleSim_StepMultiple(VehicleSimulation* sim, const DriverInputs* inputs, int num_steps)

// Output - All fully implemented
void VehicleSim_GetRenderData(const VehicleSimulation* sim, VehicleRenderData* out)
void VehicleSim_GetTelemetry(const VehicleSimulation* sim, TelemetryFrame* out)
void VehicleSim_RegisterTelemetryCallback(VehicleSimulation* sim, TelemetryCallback cb, void* user_data)

// Asset Management
int VehicleSim_LoadVehicle(VehicleSimulation* sim, const char* config_path)
void VehicleSim_GetVehicleParameters(const VehicleSimulation* sim, VehicleParameters* out)
```

**Debug harness (`unityInterface/src/debug_main.c`):**

> The old internal `Sim*` API (`sim_create`, `sim_step`, `sim_get_state`, …) and its
> source files (`src/simulation/sim.c`, `simulation_loop.c`, `telemetry.c`) have been
> **removed** (February 2026). `debug_main.c` now calls `VehicleSim_*` directly —
> `unity_api.c` is compiled statically into the exe. Every code path exercised by
> the debug harness is the exact same code that ships in `racing_sim.dll`.

```c
// debug_main.c startup (mirrors Unity Start())
VehicleSimulation* sim = VehicleSim_Create(0.001);
VehicleSim_SetLogCallback(sim, my_log_cb, NULL);
VehicleSim_LoadVehicle(sim, "data/vehicles/TBReCar.txt");
VehicleSim_LoadTrack(sim, "data/tracks/skidpad.txt");
VehicleSim_Initialize(sim);

// debug_main.c step loop (mirrors Unity FixedUpdate())
DriverInputs inputs = { .throttle=1.0, .gear=1 };
VehicleSim_Step(sim, &inputs);

// debug_main.c output (mirrors Unity Update())
TelemetryFrame frame;  VehicleSim_GetTelemetry(sim, &frame);
VehicleRenderData rd;  VehicleSim_GetRenderData(sim, &rd);
```

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

#include "core/math/math_base.h"

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

### Naming Conventions

**Functions:** `module_verb_noun` (e.g., `vehicle_update_kinematics`)
**Types:** `PascalCase` (e.g., `SprungMass`, `TireModel`)
**Variables:** `snake_case` (e.g., `wheel_speed`, `tire_force`)
**Constants:** `UPPER_CASE` (e.g., `VDE_PI`, `VDE_GRAVITY`)

### Comments

- Use `//` for inline comments
- Use `/** */` blocks for function documentation
- Always reference Guiggiani sections where applicable
- Example: `// Guiggiani Section 3.7.2 - Lateral load transfer`

---

## 🗺️ Project Structure

```
FullCarSim/
├── include/              # Headers
│   ├── core/            # Math, integrators, physics, utils
│   ├── vehicle/         # Three-equation structure + components
│   ├── tire_models/     # Magic Formula, Brush
│   ├── track/           # Track geometry
│   └── input/           # Controls
│
├── src/                 # Implementations (mirrors include/)
├── unityInterface/      # DLL build + debug harness
├── data/               # Configs, tracks, vehicles
├── build/              # Build outputs
│
├── README.md           # Project overview + build guide
├── AI_CONTEXT.md       # This file - architecture + standards
└── QUICK_REFERENCE.md  # Quick lookup for Guiggiani refs
```

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
| `math_base.h/c` | Core types, constants, scalar ops | `vde_real`, constants, math wrappers |
| `vec3.h/c` | 3D vectors | Add, sub, dot, cross, normalize |
| `mat3.h/c` | 3x3 matrices | Multiply, determinant, inverse, transforms |
| `quat.h/c` | Quaternions | SLERP, rotation, conversion, integration |
| `frames.h/c` | Rigid transforms | Body↔world transforms, composition |

**Scalar operations** (`math_base.h`):
```c
// Core utilities
vde_abs(x)           // Absolute value
vde_clamp(v, lo, hi) // Clamp to range
vde_min(a, b)        // Minimum
vde_max(a, b)        // Maximum
vde_sign(x)          // Sign function (-1, 0, 1)
vde_square(x)        // x * x
vde_lerp(a, b, t)    // Linear interpolation

// Type-safe math wrappers (for vde_real precision)
vde_sqrt(x)          // Square root
vde_sin(x), vde_cos(x), vde_tan(x)     // Trigonometry
vde_asin(x), vde_acos(x), vde_atan(x)  // Inverse trig
vde_atan2(y, x)      // Two-argument arctangent
vde_pow(x, y)        // Power
vde_exp(x), vde_log(x)                  // Exponential/logarithm
vde_floor(x), vde_ceil(x)               // Rounding

// Validation
vde_isfinite(x)      // Check for NaN/Inf
vde_approx(a, b, tol) // Approximate equality
```

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
VDE_HALF_PI        // π/2
VDE_TWO_PI         // 2π
VDE_DEG2RAD        // Degrees to radians
VDE_RAD2DEG        // Radians to degrees
VDE_EPS            // Small number (1e-9)
VDE_SQRT_EPS       // Square root of epsilon (1e-6)
```

**IMPORTANT:** Always use math library wrappers (e.g., `vde_sqrt()`, `vde_abs()`) instead of bare C math functions (e.g., `sqrt()`, `fabs()`) for type safety and consistency.

---

### Numerical Integrators (`include/core/integrator/`, `src/core/integrator/`) - ✅ COMPLETE

**Status:** Production-ready, three integrator implementations available

| File | Purpose | Order | Key Features |
|------|---------|-------|--------------|
| `integrator_base.h/c` | Explicit Euler | 1st order | Simple, fast, suitable for testing |
| `runge_kutta4.h/c` | RK4 | 4th order | High accuracy, 4 derivative evaluations |
| `semi_implicit_euler.h/c` | Symplectic Euler | 1st order | Better energy conservation |

**Common API:**
```c
// All integrators follow this pattern
Integrator* integrator_create(void);
void integrator_destroy(Integrator* integrator);
void integrator_step(
    Integrator* integrator,
    vde_real* state,
    int n,
    vde_real dt,
    DerivativeFunc deriv_func,
    void* user_data
);
```

**Key features:**
- Dynamic memory allocation for state vectors
- Automatic resizing based on state vector size
- Numerical stability checks (`vde_isfinite`)
- Generic interface via function pointers
- Defensive programming with null/bounds checking

**Usage notes:**
- **Explicit Euler**: Best for initial testing, simplest implementation
- **RK4**: Recommended for production, best accuracy-to-cost ratio
- **Semi-implicit Euler**: Currently implements explicit Euler for generic state vectors; specialized vehicle dynamics integrator should handle position/velocity ordering at the equations of motion level

---

### Three-Equation Structure Headers - ✅ API COMPLETE + SCAFFOLDED IMPLEMENTATIONS

Headers make Guiggiani's methodology explicit; source files exist with correct structure but placeholder logic pending full Vehicle expansion.

| Header | Source File | Status | Guiggiani Ref |
|--------|-------------|--------|---------------|
| `vehicle_model.h` | `vehicle_model.c` | 🚧 ALPHA - state retrieval working, full step pending | Ch 3, Sec 3.11-3.12 |
| `vehicle_congruence.h` | `vehicle_congruence.c` | 🚧 SCAFFOLDED - Ackermann impl; other fns return zeros | Ch 3, Sec 3.2 |
| `vehicle_constitutive.h` | `vehicle_constitutive.c` | 🚧 SCAFFOLDED - linear suspension impl; tires return zeros | Ch 3, Sec 3.3 |
| `vehicle_equilibrium.h` | `vehicle_equilibrium.c` | 🚧 SCAFFOLDED - load transfer impl; force assembly returns zeros | Ch 3, Sec 3.4-3.6, 3.11 |

**Implementation Notes:**
- All three source files created with proper Guiggiani structure and section headers
- Each includes USER_DECISION and POST-ALPHA TODO comments for expansion choices
- Require full 3D Vehicle structure (position, orientation, velocity, components) to complete
- Integration points clearly documented for future expansion

---

### Tire Models (`tire_models/`) - ✅ FULLY IMPLEMENTED

**Status:** All tire model implementations complete with proper Guiggiani methodology

| Component | Header | Source | Status |
|-----------|--------|--------|--------|
| Tire Utilities | `tire_utilities.h` | `tire_utilities.c` | ✅ COMPLETE - All functions implemented |
| Magic Formula | `magic_formula.h` | `magic_formula.c` | ✅ COMPLETE - Full MF implementation |
| Brush Model | `brush_models.h` | `brush_models.c` | ✅ IMPLEMENTED - Simplified with upgrade path |
| Tire Interface | `tire.h` | `tire.c` | ✅ COMPLETE - Integrates both models |

**Implementation highlights:**
- **Tire utilities**: Force transformations, contact velocity, slip calculations
- **Magic Formula**: Full Pacejka model with longitudinal/lateral forces, combined slip
- **Brush model**: Simplified linear+saturation with USER_DECISION for full implementation
- **Tire interface**: Proper model selection and dispatch

---

### Vehicle Components (`vehicle/`) - ✅ API DEFINED + PARTIAL IMPLEMENTATIONS

**Status:** All component APIs fully defined with comprehensive Guiggiani references, implementations vary

**Complete headers with implementations:**
- `tire.h` / `tire.c` - ✅ COMPLETE - Tire component with model integration
- `wheel.h` / `wheel.c` - ✅ MOSTLY COMPLETE - State management, slip computation stubs
- `suspension.h` / `suspension.c` - ✅ MOSTLY COMPLETE - Linear spring/damper model
- `brakes.h` / `brakes.c` - ✅ MOSTLY COMPLETE - Brake torque calculations
- `aerodynamics.h` / `aerodynamics.c` - ✅ MOSTLY COMPLETE - Basic aero forces
- `steering.h` / `steering.c` - 📝 PARTIAL - API defined, basic implementation
- `driveline.h` / `driveline.c` - 📝 PARTIAL - API defined, basic implementation
- `sprung_mass.h` / `sprung_mass.c` - 📝 PARTIAL - API defined
- `unsprung_mass.h` / `unsprung_mass.c` - 📝 PARTIAL - API defined
- `vehicle.h` / `vehicle.c` - ✅ WORKING 2D - Simple dynamics fully functional, expansion to 3D planned
- `vehicle_parameters.h` / `vehicle_parameters.c` - 📝 PARTIAL - Parameter management

**Implementation status:** Most components have lifecycle, getters/setters, and placeholder force computations. Vehicle.c has working 2D dynamics with proper input handling. State retrieval fully implemented. TODOs documented for expansion to full 3D Vehicle structure.

---

### Core Physics (`core/physics/`) - ✅ API DEFINED + PARTIAL IMPLEMENTATIONS

- `equations_of_motion.h` / `.c` - ✅ PARTIAL - Structure/allocation complete, solver stubs (Guiggiani 3.4-3.6, Ch 9)
- `dynamics_solver.h` / `.c` - 📝 PARTIAL - Main dynamics solver API defined (Guiggiani 3.11)
- `constraints.h` / `.c` - 📝 PARTIAL - Constraint solver API defined

**Implementation status:** EOM structure complete, solver logic requires Vehicle expansion

---

### Unity API Layer (`unity_api.h/c`) - ✅ FULLY WORKING

- `unity_api.h` - Complete DLL interface definition ✅ COMPLETE
- `unity_api.c` - Full implementation of all API functions ✅ COMPLETE

**Implemented functionality:**
- ✅ Lifecycle management (Create, Destroy, Initialize, Reset)
- ✅ Simulation stepping with inputs (VehicleSim_Step takes DriverInputs parameter)
- ✅ Complete render data output (chassis + 4 wheels with positions, orientations)
- ✅ Full telemetry system (73+ channels, callbacks, recording)
- ✅ Vehicle parameters (TBRe FSAE-style defaults)
- ✅ State validation and performance statistics
- ✅ Input handling (inputs passed directly to step function)
- ✅ 2D to 3D state conversion (2D vehicle state → 3D VehicleState structure)

**Working simulation flow:**
1. Create simulation with timestep
2. Initialize vehicle and track
3. Call VehicleSim_Step(sim, &inputs) each physics frame
4. Retrieve VehicleRenderData for Unity visualization
5. Retrieve TelemetryFrame for data logging

### Simulation System - 🗑️ FULLY REMOVED (February 2026)

The entire `simulation/` subtree has been deleted:
- `src/simulation/sim.c`, `simulation_loop.c`, `telemetry.c` — dead code, no callers
- `include/simulation/simulation_config.h`, `simulation_loop.h`, `telemetry.h` — orphaned headers

**Reason:** `unity_api.c` is a complete, self-contained simulation loop.
A separate internal `Sim*` API created two divergent code paths with no active users.
The debug harness (`debug_main.c`) now calls `VehicleSim_*` directly.

---

### Other Subsystems - ✅ API DEFINED + IMPLEMENTATIONS VARY

All have complete APIs with Guiggiani references and varying implementation levels:

- **Track** (`track/`) - 📝 PARTIAL - Surface, geometry, friction APIs defined (Guiggiani Ch 5)
  - `track_surface.c`, `track_geometry.c`, `friction_map.c` - Basic structures
- **Input** (`input/`) - ✅ MOSTLY COMPLETE - Control input system implemented
  - `control.c` - Input filtering, getters/setters, state management complete
- **Integrators** (`core/integrator/`) - ✅ COMPLETE - All three methods fully implemented
  - `integrator_base.c` - Explicit Euler complete
  - `semi_implicit_euler.c` - Semi-implicit Euler complete  
  - `runge_kutta4.c` - RK4 complete
- **Utils** (`core/utils/`) - ✅ MOSTLY COMPLETE - Logger with levels and callbacks
  - `logger.c` - Log levels, file output, callbacks, filtering implemented

**Implementation summary:**
- ✅ **Integrators** - COMPLETE (all three methods production-ready)
- ✅ **Input/Control** - MOSTLY COMPLETE - Full input management system
- ✅ **Logger** - MOSTLY COMPLETE - Complete logging infrastructure
- 📝 **Track** - PARTIAL - APIs defined, awaiting implementation

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

- ✅ **unity_api.c** - FULLY IMPLEMENTED - Complete working DLL with all functions
- ✅ **debug_main.c** - REWORKED - Drives entirely through VehicleSim_* (unity_api.c path)
- 🗑️ **simulation/** - FULLY REMOVED - src/ and include/ both deleted (dead code)
- ✅ **vehicle.c** - WORKING 2D SIMULATION - Simple dynamics fully functional
- ✅ **State retrieval** - vehicle_model_get_state() converts 2D→3D properly
- ✅ **Render data** - All output structures populated with real simulation data
- ✅ **Telemetry** - Complete telemetry system with 73+ channels
- ✅ **Input handling** - VehicleSim_Step() takes inputs parameter (NEW API)
- ✅ **core/math/** - Complete (headers + implementations)
- ✅ **core/integrator/** - Complete (headers + implementations)
- ✅ **tire_models/** - Complete (Magic Formula + simplified Brush model)
- 🚧 **Three-equation structure** - Scaffolded (congruence.c, constitutive.c, equilibrium.c); key logic returns zeros until Vehicle is expanded to full 3D
- ✅ **vehicle/tire.c** - Complete (integrates tire models)
- ✅ **All subsystem headers** - Complete API definitions with Guiggiani refs
- ✅ **Comprehensive documentation** - README files with examples
- ✅ **Input/Logger/Utils** - Mostly complete implementations
- 🚧 **Vehicle components** - Partial implementations with expansion points
- 📝 **Core physics solvers** - Structures defined, integration pending
- 📝 **Track subsystem** - APIs defined, basic structures

**Major Progress (February 18, 2026):**
- ✅ Unity API fully implemented and working end-to-end
- ✅ VehicleSim_Step() reworked to take inputs as parameter (better API design)
- ✅ Complete 2D vehicle simulation running with proper state retrieval
- ✅ All render data and telemetry structures populated with real data
- ✅ 2D to 3D state conversion working (position, orientation, velocity)
- ✅ Wheel positions calculated and transformed to world space
- ✅ All tire model implementations complete
- ✅ Three-equation structure source files created
- ✅ Vehicle component implementations with USER_DECISION points
- ✅ Complete tire force computation pipeline

**Major Progress (February 19, 2026):**
- 🗑️ Deleted `src/simulation/` — sim.c, simulation_loop.c, telemetry.c (dead code, no callers)
- 🗑️ Deleted `include/simulation/` — simulation_config.h, simulation_loop.h, telemetry.h (orphaned headers)
- ✅ Rewrote `debug_main.c` to drive entirely through `VehicleSim_*` Unity API
- ✅ `unity_api.c` now compiles statically into `sim_debug.exe` — identical code path to DLL
- ✅ Debug harness now exercises: `SetLogCallback`, `TelemetryCallback`, `GetRenderData`,
  `VehicleSim_Validate`, `VehicleSim_GetStats`, `VehicleSim_GetDiagnostics`, `Reset`
- ✅ Removed stale `#include "simulation/simulation_loop.h"` from `unity_api.c`
- ✅ Updated `CMakeLists.txt`: removed dead sources, added `unity_api.c` to `DEBUG_SRC`,
  replaced `STATIC_BUILD` define with `BUILDING_DLL` for consistent `UNITY_API` expansion
- ✅ Default debug timestep changed from 0.02 s (50 Hz) to 0.001 s (1 kHz) to match DLL rate
- ✅ Updated `DEBUG_README.md` and `unityInterface/QUICK_REFERENCE.md`

**Current State:**
The DLL can now run a complete driving simulation! Simple 2D dynamics are fully functional with:
- Throttle control (5 m/s² acceleration)
- Brake control (10 m/s² deceleration)
- Steering control (yaw rate change)
- Position and velocity tracking
- Proper state output to Unity

**Next Priorities:**
1. Build and test DLL with Unity integration
2. Expand Vehicle structure to full 3D state (6DOF rigid body)
3. Complete integration of three-equation structure with vehicle components
4. Implement full dynamics solver pipeline with tire forces
5. Track geometry and surface implementation

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
   
   #include "core/math/math_base.h"
   
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
void set_speed(Vehicle* v, vde_real speed) {
    v->speed = speed;  // Could be negative or NaN!
}
```

### ✅ DO THIS INSTEAD:

```c
// Correct: Using vde_real
vde_real position = (vde_real)10.0;

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
1. **Unity DLL API:** See `include/unity_api.h` (THE Unity boundary — ~500 lines of documentation)
2. **Asset management:** See `unity_api.h` - VehicleSim_LoadVehicle / VehicleSim_LoadTrack
3. **Telemetry system:** See `unity_api.h` - TelemetryFrame struct and TelemetryCallback

**Internal Physics:**
4. **Project organization:** See `include/README.md`
5. **Vehicle components:** See `include/vehicle/README.md`
6. **Tire models:** See `include/tire_models/README.md` (extensive guide)
7. **Physics solvers:** See `include/core/physics/README.md`
8. **Coding standards:** See `AI_CONTEXT.md` > Standards Summary section
9. **Implementation priorities:** See `AI_CONTEXT.md` > Next Priorities
10. **Build instructions:** See `README.md`
11. **Math API reference:** Look at `include/core/math/*.h` headers
12. **Component APIs:** All headers in `include/` have complete function APIs
13. **Working examples:** Check `src/vehicle/vehicle.c`, `src/unity_api.c`, `unityInterface/src/debug_main.c`
14. **Guiggiani references:** Every header has chapter/section annotations

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
- ✅ **unity_api.h** - Complete Unity interface definition
- ✅ **unity_api.c** - FULLY IMPLEMENTED - All functions working, simulation runs end-to-end
- ✅ **vehicle.c** - WORKING 2D SIMULATION - Simple dynamics functional
- ✅ **vehicle_model.c** - State retrieval and conversion working
- ✅ **core/math/** - Complete (headers + implementations)
- ✅ **core/integrator/** - Complete (headers + implementations)
- ✅ **tire_models/** - Complete implementations (Magic Formula, Brush, utilities)
- ✅ **tire.c** - Complete tire component with model integration
- 🚧 **vehicle_congruence.c** - Scaffolded; Ackermann geometry works, rest returns zeros
- 🚧 **vehicle_constitutive.c** - Scaffolded; linear suspension works, tire forces return zeros
- 🚧 **vehicle_equilibrium.c** - Scaffolded; load transfer works, force assembly returns zeros
- ✅ **All headers** - Complete API definitions with Guiggiani refs
- ✅ **Documentation** - Comprehensive README files across include/
- 🚧 **vehicle components** - Partial implementations (wheel, suspension, brakes, aero)
- 📝 **core/physics/** - Structures defined, integration pending
- 📝 **Track subsystem** - APIs defined, basic structures

### Documentation Quick Reference:
- `unity_api.h` - ⭐ THE DLL boundary with comprehensive inline docs
- `include/README.md` - Overview of entire header structure
- `include/vehicle/README.md` - Vehicle components guide
- `include/tire_models/README.md` - Detailed tire model guide
- `include/core/physics/README.md` - Physics solvers guide

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
7. **READ** `AI_CONTEXT.md` > Standards Summary section first
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
