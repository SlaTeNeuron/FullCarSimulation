# FullCarSim - Vehicle Dynamics Engine

A high-fidelity vehicle dynamics simulation engine written in modern C11, designed for racing applications with GPU compatibility in mind.

**Status:** 🎯 ALPHA - DLL Fully Working, Ready for Unity Integration Testing  
**Current Version:** 0.4.0-alpha (Unity API fully implemented, simulation running end-to-end)  
**Last Updated:** February 18, 2026

---

## 🚀 Quick Start

**First time here?** Read **[AI_CONTEXT.md](AI_CONTEXT.md)** for architecture and coding standards.

### Building for Alpha

**Prerequisites:**
- Windows 10/11
- Visual Studio 2019 or later (with C++ tools)
- CMake 3.15+

**Build Instructions:**
```powershell
# Navigate to Unity interface directory
cd unityInterface

# Configure and build (generates DLL + debug executable)
.\build_debug.bat

# Run debug test executable
.\run_debug.bat
```

---

## 📚 Documentation

- **[README.md](README.md)** - This file - project overview and build guide
- **[AI_CONTEXT.md](AI_CONTEXT.md)** - For AI assistants - architecture, implementation status, coding standards
- **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** - Quick lookup for common tasks and Guiggiani references

---

## 🏗️ Architecture Overview

### Core Principles

- **Language:** C11 (modern C, not C++)
- **Methodology:** Guiggiani's "The Science of Vehicle Dynamics" - three-equation structure
- **Precision:** Configurable via `vde_real` (defaults to double)
- **Platform:** Cross-platform (Windows/Linux/macOS)
- **Target:** DLL for Unity integration

### Guiggiani's Three-Equation Structure

The core simulation follows Section 3.12 of Guiggiani:

1. **CONGRUENCE** (Kinematic) - How vehicle parts move
   - Wheel velocities, tire slips (σ, α), suspension rates
   
2. **CONSTITUTIVE** (Component Behavior) - Forces from states
   - Tire forces, spring forces, damper forces, aero forces
   
3. **EQUILIBRIUM** (Force Balance) - Newton's laws
   - F = m·a, M = I·α, load transfers

### Module Status

| Module | Status | Description |
|--------|--------|-------------|
| **unity_api.c** | ✅ Complete | Full DLL implementation, all functions working |
| **core/math/** | ✅ Complete | 3D vectors, quaternions, matrices, frames |
| **core/integrator/** | ✅ Complete | Euler, RK4, Semi-implicit Euler |
| **tire_models/** | ✅ Complete | Magic Formula + Brush tire models |
| **vehicle/** | 👍 Working | Simple 2D dynamics, needs 3D expansion |
| **track/** | 📝 Stub | Track geometry and surface |
| **simulation/** | ✅ Complete | Full integration with Unity API |

---

## 🎮 Unity Integration

### Two-Layer API Architecture

**Layer 1: Internal C API** (`simulation/sim.c` + `simulation/simulation_config.h`)
- Used by: Standalone debug executable (`sim_debug.exe`)
- Functions: `sim_create()`, `sim_step()`, `sim_set_inputs()`, etc.
- Purpose: Simple C interface for internal testing

**Layer 2: Unity DLL API** (`src/unity_api.c` + `include/unity_api.h`)
- Used by: Unity C# through P/Invoke
- Status: ✅ FULLY IMPLEMENTED - All functions working, simulation runs end-to-end
- Functions: `VehicleSim_Create()`, `VehicleSim_Step()`, `VehicleSim_GetRenderData()`, etc.
- Purpose: Complete DLL boundary for Unity with comprehensive telemetry

**Key API Change:** `VehicleSim_Step()` now takes `DriverInputs` as a parameter for better efficiency:
```c
void VehicleSim_Step(VehicleSimulation* sim, const DriverInputs* inputs)
```
This eliminates the need for separate `SetInputs()` calls.

### Unity DLL Functions

```c
// Lifecycle
VehicleSimulation* VehicleSim_Create(double timestep)
void VehicleSim_Destroy(VehicleSimulation* sim)
int VehicleSim_Initialize(VehicleSimulation* sim)
void VehicleSim_Reset(VehicleSimulation* sim)

// Simulation Control - NEW: Step takes inputs directly
void VehicleSim_Step(VehicleSimulation* sim, const DriverInputs* inputs)
void VehicleSim_StepMultiple(VehicleSimulation* sim, const DriverInputs* inputs, int num_steps)

// Output - All fully implemented
void VehicleSim_GetRenderData(const VehicleSimulation* sim, VehicleRenderData* out)
void VehicleSim_GetTelemetry(const VehicleSimulation* sim, TelemetryFrame* out)
void VehicleSim_RegisterTelemetryCallback(VehicleSimulation* sim, TelemetryCallback cb, void* data)

// Asset Management
int VehicleSim_LoadVehicle(VehicleSimulation* sim, const char* config_path)
void VehicleSim_GetVehicleParameters(const VehicleSimulation* sim, VehicleParameters* out)
void VehicleSim_GetTrackInfo(const VehicleSimulation* sim, TrackInfo* out)

// Validation & Stats
void VehicleSim_Validate(const VehicleSimulation* sim, ValidationResult* out)
void VehicleSim_GetStats(const VehicleSimulation* sim, SimulationStats* out)
```

### C# Interop Example

```csharp
[StructLayout(LayoutKind.Sequential)]
public struct DriverInputs {
    public double throttle;
    public double brake;
    public double steering;
    public double clutch;
    public int gear;
}

[DllImport("racing_sim")]
private static extern IntPtr VehicleSim_Create(double timestep);

[DllImport("racing_sim")]
private static extern void VehicleSim_Step(IntPtr sim, ref DriverInputs inputs);

[DllImport("racing_sim")]
private static extern void VehicleSim_GetRenderData(IntPtr sim, out VehicleRenderData data);

// Usage in Unity
void FixedUpdate() {
    DriverInputs inputs = new DriverInputs {
        throttle = Input.GetAxis("Throttle"),
        brake = Input.GetAxis("Brake"),
        steering = Input.GetAxis("Steering"),
        clutch = 0.0,
        gear = 1
    };
    VehicleSim_Step(simHandle, ref inputs);
}
```

See `include/unity_api.h` for complete API reference with 400+ lines of documentation.

---

## 🔧 Technologies

### Math Library

Complete 3D mathematics with type-safe wrappers for all operations:

**Scalar Operations** (`math_base.h`):
- Core: `vde_abs()`, `vde_clamp()`, `vde_min()`, `vde_max()`, `vde_sign()`
- Interpolation: `vde_lerp()`, `vde_square()`
- Trigonometry: `vde_sin()`, `vde_cos()`, `vde_tan()`, `vde_atan2()`
- Math functions: `vde_sqrt()`, `vde_pow()`, `vde_exp()`, `vde_log()`
- Validation: `vde_isfinite()`, `vde_approx()`

**Vector Types** (`vec3.h`):
- Vectors (`vde_vec3`) - add, subtract, dot, cross, normalize, scale
- Operations: `vde_vec3_norm()`, `vde_vec3_norm2()`, `vde_vec3_madd()`

**Rotation Types**:
- Matrices (`vde_mat3`) - multiply, inverse, transpose, transforms
- Quaternions (`vde_quat`) - rotation, SLERP, integration
- Frames (`vde_frame`) - rigid body transforms

**Example:**
```c
vde_vec3 position = vde_vec3_make(1.0, 2.0, 3.0);
vde_quat rotation = vde_quat_from_axis_angle(&axis, angle);
vde_vec3 rotated = vde_quat_rotate(&rotation, &position);

// Use type-safe math wrappers
vde_real speed = vde_sqrt(vde_square(vx) + vde_square(vy));
vde_real angle = vde_atan2(vy, vx);
vde_real value = vde_clamp(input, 0.0, 1.0);
```

### Internal C API (Debug/Testing)

```c
// Used by standalone test executable
Sim* sim = sim_create(0.02);  // 50 Hz
sim_start(sim);
sim_set_inputs(sim, throttle, brake, steer);
sim_step(sim);
sim_get_state(sim, &state);
sim_destroy(sim);
```

### Tire Models

- **Magic Formula** (Pacejka) - Empirical, industry standard
- **Brush Model** - Physical with transient effects

### Numerical Integration

- **Explicit Euler** - Fast, 1st order
- **Runge-Kutta 4** - Accurate, 4th order
- **Semi-implicit Euler** - Stable for stiff systems

---

## 📦 Project Structure

```
FullCarSim/
├── include/              # Public API headers
│   ├── math/            # ✅ 3D math library (complete)
│   ├── vehicle/         # 🚧 Vehicle model
│   ├── simulation/      # ✅ Simulation core
│   ├── control/         # 📝 Control systems
│   ├── sensors/         # 📝 Sensor suite
│   └── track/           # 📝 Track/environment
│
├── src/                  # Implementation files (mirrors include/)
│
├── unityInterface/       # Unity/External API
│   ├── include/         # Simplified C API
│   ├── src/             # Implementation + debug harness
│   └── build/           # CMake build files
│
├── data/                 # Runtime configuration
│   ├── configs/         # Simulation configs
│   ├── tracks/          # Track definitions
│   └── vehicles/        # Vehicle parameters
│
├── build/                # Build output directory
│
├── AI_CONTEXT.md         # 🤖 Architecture, standards, and implementation status
└── README.md             # 📄 This file
```

---

## 🎯 Post-Alpha Roadmap

1. **Expand Vehicle to full 3D** - Position, orientation, 6DOF velocities (Priority 1)
2. **Transient brush model** - Bristle dynamics (Guiggiani Ch. 10)
3. **Full load transfers** - Sprung/unsprung masses (Section 3.10)
4. **Inerter suspension** - Advanced damping (Section 8.2.1)
5. **Caching optimization** - Dirty flags for constitutive queries
6. **CFD aerodynamics** - Lookup tables for ground effect

---

## 💡 Design Philosophy

### Why C11?

- **Performance:** Direct hardware access, minimal overhead
- **Portability:** Works everywhere (embedded to HPC)
- **GPU-ready:** Easy to port to CUDA/OpenCL
- **Deterministic:** Predictable behavior for simulation

### Why `vde_real`?

Floating-point abstraction enables:
- Quick precision switching (double ↔ float)
- GPU optimization (float on GPU, double on CPU)
- Numerical stability tuning
- Future-proof architecture

### Why Defensive Programming?

Racing simulations must be robust:
- Handle edge cases gracefully
- Recover from numerical issues
- Validate all inputs
- Fail safely, never crash

---

## 🤝 Contributing

**Before coding:**
1. Read [AI_CONTEXT.md](AI_CONTEXT.md) for architecture and standards
2. Use `vde_real` for all floating-point
3. Use math library types (`vde_vec3`, `vde_quat`, `vde_mat3`)
4. Add defensive null checks
5. Follow section organization with `//-------------------------`

---

## 🔄 Changelog
- Architecture → Update `AI_CONTEXT.md`
- Standards → Update `AI_CONTEXT.md` > Standards Summary
- Key patterns → Update `AI_CONTEXT.md`
- Build process → Update this README

**Always update the "Last Updated" date at the top of modified docs.**

---

**Ready to dive in?** Start with [AI_CONTEXT.md](AI_CONTEXT.md) 🚀
