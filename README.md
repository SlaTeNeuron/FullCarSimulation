# FullCarSim - Vehicle Dynamics Engine

A high-fidelity vehicle dynamics simulation engine written in modern C11, designed for racing applications with GPU compatibility in mind.

**Status:** 🚧 In Active Development  
**Current Version:** 0.2.0 (Math system complete, basic simulation working)  
**Last Updated:** February 13, 2026

---

## 🚀 Quick Start

### For Developers

**First time here?** Read these in order:

1. **[AI_CONTEXT.md](AI_CONTEXT.md)** - 2-minute overview (essential for AI assistants)
2. **[CODING_STANDARDS.md](CODING_STANDARDS.md)** - Complete style guide
3. **[copilotPlan.txt](copilotPlan.txt)** - Project structure & roadmap

### Building

```bash
cd unityInterface
build_debug.bat      # Windows
```

### Running Tests

```bash
cd unityInterface
run_debug.bat        # Run automated test suite
```

---

## 📚 Documentation Index

| Document | Purpose | Audience |
|----------|---------|----------|
| [AI_CONTEXT.md](AI_CONTEXT.md) | Quick reference, common tasks | AI assistants, new developers |
| [CODING_STANDARDS.md](CODING_STANDARDS.md) | Complete style guide & principles | All developers |
| [copilotPlan.txt](copilotPlan.txt) | Architecture & expansion plan | Architects, project leads |
| [README.md](README.md) | This file - project overview | Everyone |

---

## 🏗️ Architecture Overview

### Core Principles

- **Language:** C11 (modern C, not C++)
- **Precision:** Configurable via `vde_real` (defaults to double)
- **Platform:** Cross-platform (Windows/Linux/macOS)
- **Future-proof:** Prepared for GPU deployment (CUDA/OpenCL)

### Module Status

| Module | Status | Description |
|--------|--------|-------------|
| **math/** | ✅ Complete | 3D math library (vec3, quat, mat3, frames) |
| **simulation/** | ✅ Solid | Core simulation loop & API |
| **vehicle/** | 🚧 Basic | Simple 2D model (expanding to 6DOF) |
| **control/** | 📝 Placeholder | Control systems (planned) |
| **sensors/** | 📝 Placeholder | Sensor suite (planned) |
| **track/** | 📝 Placeholder | Track/environment (planned) |

---

## 🔧 Key Technologies

### Math Library

Complete 3D mathematics with:
- Vectors (`vde_vec3`) - add, subtract, dot, cross, normalize
- Matrices (`vde_mat3`) - multiply, inverse, transpose, transforms
- Quaternions (`vde_quat`) - rotation, SLERP, integration
- Frames (`vde_frame`) - rigid body transforms

**Example:**
```c
vde_vec3 position = vde_vec3_make(1.0, 2.0, 3.0);
vde_quat rotation = vde_quat_from_axis_angle(&axis, angle);
vde_vec3 rotated = vde_quat_rotate(&rotation, &position);
```

### Simulation Core

Flexible, high-performance simulation engine:
```c
Sim* sim = sim_create(0.02);  // 50 Hz
sim_set_inputs(sim, throttle, brake, steer);
sim_step(sim);  // Advance one timestep
sim_get_state(sim, &state);  // Extract telemetry
```

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
├── AI_CONTEXT.md         # 🤖 Quick reference for AI/developers
├── CODING_STANDARDS.md   # 📖 Complete coding standards
├── copilotPlan.txt       # 🗺️ Project roadmap
└── README.md             # 📄 This file
```

---

## 🎯 Development Roadmap

See [copilotPlan.txt](copilotPlan.txt) for detailed expansion plan.

**Immediate priorities:**

1. **Phase 1:** Upgrade to 6DOF rigid body dynamics
2. **Phase 2:** Implement tire & suspension models
3. **Phase 3:** Add track geometry & surface properties
4. **Phase 4:** Complete vehicle subsystems (aero, driveline, brakes)
5. **Phase 5:** Control systems & driver models
6. **Phase 6:** Sensor suite & enhanced telemetry

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

### Before Writing Code

1. Read [AI_CONTEXT.md](AI_CONTEXT.md) (2-minute crash course)
2. Review [CODING_STANDARDS.md](CODING_STANDARDS.md) Quick Reference
3. Check [copilotPlan.txt](copilotPlan.txt) for planned work
4. Follow existing patterns in the codebase

### Code Requirements

All code must:
- Use `vde_real` (never bare `double`/`float`)
- Use math library types (`vde_vec3`, `vde_quat`, etc.)
- Include defensive null/validation checks
- Follow section organization (`//-------------------------`)
- Use `#pragma once` (never `#ifndef` guards)
- Update documentation if adding new patterns

### Pull Request Checklist

- [ ] Code follows CODING_STANDARDS.md
- [ ] All defensive checks included
- [ ] Uses `vde_real` consistently
- [ ] Section headers added
- [ ] Builds without errors
- [ ] Updated relevant documentation
- [ ] Added to copilotPlan.txt if new module

---

## 🐛 Known Issues

- Vehicle model is simplified 2D (6DOF upgrade in progress)
- No tire model yet (using basic force model)
- Track is flat (geometry system planned)

See [copilotPlan.txt](copilotPlan.txt) for complete feature status.

---

## 📝 License

*[Add license information here]*

---

## 📞 Contact

*[Add contact information here]*

---

## 🔄 Maintenance Notes

**⚠️ Keep documentation synchronized!**

When you change:
- Architecture → Update copilotPlan.txt
- Standards → Update CODING_STANDARDS.md
- Key patterns → Update AI_CONTEXT.md
- Build process → Update this README

**Always update the "Last Updated" date at the top of modified docs.**

---

**Ready to dive in?** Start with [AI_CONTEXT.md](AI_CONTEXT.md) 🚀
