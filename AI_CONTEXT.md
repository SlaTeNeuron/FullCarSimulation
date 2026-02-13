# AI Assistant Context - FullCarSim

**🤖 FOR AI ASSISTANTS: Read this FIRST before making any code changes**

**Last Updated:** February 13, 2026  
**Codebase Status:** Math system complete, basic simulation working, vehicle systems under development

---

## ⚡ Quick Start (30-Second Overview)

This is a **vehicle dynamics engine** written in **C11** for racing simulation.

**Key Facts:**
- Language: C11 (not C++)
- Precision: Configurable via `vde_real` (defaults to `double`)
- Structure: `include/` + `src/` split
- Math: Complete 3D math library in `include/math/` 
- Status: Basic 2D sim working, expanding to full 6DOF physics

**Before coding anything:**
1. Read the [Standards Summary](#standards-summary) below
2. Check [What's Implemented](#whats-implemented)
3. Follow the patterns in [Code Examples](#code-examples)

---

## 📋 Standards Summary

### Critical Rules (Never Break These)

1. **Use `#pragma once`** - Never `#ifndef` guards
2. **Use `vde_real`** - Never bare `double` or `float`
3. **Use math types** - `vde_vec3`, `vde_quat`, `vde_mat3`, `vde_frame`
4. **Defensive programming** - Always check nulls, validate inputs
5. **Section headers** - Organize code with `//-------------------------`
6. **Include math_base.h** - Always include for `vde_real` and constants

### Quick Style Reference

```c
#pragma once
// Vehicle Dynamics Engine - Module Name

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
├── include/           # Public headers
│   ├── math/         # ✅ COMPLETE - Don't modify unless necessary
│   ├── vehicle/      # 🚧 BASIC - Needs expansion
│   ├── simulation/   # ✅ SOLID - Core ready
│   ├── control/      # 📝 Placeholder
│   ├── sensors/      # 📝 Placeholder
│   └── track/        # 📝 Placeholder
├── src/              # Implementations (mirrors include/)
├── unityInterface/   # Unity/external API
├── data/             # Config files
└── build/            # Build outputs
```

---

## ✅ What's Implemented

### Math Library (`include/math/`, `src/math/`) - ✅ COMPLETE

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
VDE_EPS            // 1e-9 - epsilon for comparisons
VDE_SQRT_EPS       // 1e-6 - sqrt(epsilon)
```

**Key functions:**
```c
vde_abs(), vde_clamp(), vde_isfinite(), vde_approx()
// + all vec3, mat3, quat, frame operations
```

### Simulation Core (`include/simulation/`, `src/simulation/`) - ✅ SOLID

**Status:** Core structure ready, uses all math types

```c
Sim* sim_create(vde_real timestep);
void sim_destroy(Sim* s);
int sim_step(Sim* s);
void sim_set_inputs(Sim* s, vde_real throttle, vde_real brake, vde_real steer);
void sim_get_state(Sim* s, SimState* out);
```

Features:
- Configurable timestep
- Telemetry callbacks
- State export
- Input validation (clamped to [-1, 1])

### Vehicle (`include/vehicle/, src/vehicle/`) - 🚧 BASIC

**Status:** Simple 2D point-mass model, needs expansion to 6DOF

Current capabilities:
- 2D position (px, py)
- Velocity, yaw
- Basic throttle/brake/steering

**Next steps:** Expand to full rigid body (see copilotPlan.txt Phase 1)

### Placeholder Modules - 📝 EMPTY

- **control/** - Template added, needs implementation
- **sensors/** - Template added, needs implementation  
- **track/** - Template added, needs implementation

---

## 📝 Common Tasks

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
void set_speed(Vehicle* v, double speed) {
    v->speed = speed;  // Could be negative or NaN!
}
```

### ✅ DO THIS INSTEAD:

```c
// Correct: Using vde_real
vde_real position = (vde_real)10.0;

// Correct: pragma once
#pragma once

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

1. **Detailed standards:** See `CODING_STANDARDS.md`
2. **Project roadmap:** See `copilotPlan.txt`
3. **Build instructions:** See `README.md`
4. **Math API reference:** Look at `include/math/*.h` headers
5. **Working examples:** Check `src/vehicle/vehicle.c`, `src/simulation/sim.c`

---

## 🔄 Memory Aid: Analysis Shortcuts

**Instead of analyzing the whole codebase each time, remember:**

### Core Principles (memorize these):
1. Precision abstraction: `vde_real`
2. Math library exists: `vec3`, `quat`, `mat3`, `frame`
3. Defensive: Check nulls, validate inputs, check finite
4. Organized: Section headers, clear structure
5. Modern C: `#pragma once`, `<stdbool.h>`, `restrict`

### Quick Type Lookup:
- Scalar math → `vde_real`
- 3D position/velocity → `vde_vec3`
- Rotation → `vde_quat`
- Transform → `vde_frame`
- Matrix → `vde_mat3`

### Module Status at a Glance:
- ✅ **math/** - Complete, production-ready
- ✅ **simulation/** - Solid core structure
- 🚧 **vehicle/** - Basic, needs expansion
- 📝 **control/, sensors/, track/** - Templates only

---

## 📌 Most Important Reminders

1. **READ** `CODING_STANDARDS.md` Quick Reference section first
2. **USE** the math library - don't reinvent vectors/quaternions
3. **CHECK** nulls and validate inputs - always defensive
4. **FOLLOW** the patterns in existing code
5. **UPDATE** this file if you discover new important patterns

---

## ⚠️ Keep This Updated!

When you:
- Add new modules
- Establish new patterns
- Complete major features
- Change existing conventions

**→ Update this file with a new date stamp at the top**

This prevents future AI instances from having to re-analyze everything!

---

**Ready to code?** 
→ Quick checklist: `vde_real`? ✓ Math types? ✓ Null checks? ✓ Sections? ✓

Good luck! 🚀
