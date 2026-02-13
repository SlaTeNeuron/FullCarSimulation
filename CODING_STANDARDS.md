# FullCarSim - Coding Standards & Principles

**Version:** 1.0  
**Last Updated:** February 13, 2026  
**Status:** ✅ Active - All new code must follow these standards

---

## 📚 Table of Contents

1. [Quick Reference](#quick-reference)
2. [Language & Build](#language--build)
3. [Header Files](#header-files)
4. [Type System](#type-system)
5. [Code Organization](#code-organization)
6. [Defensive Programming](#defensive-programming)
7. [Documentation](#documentation)
8. [Naming Conventions](#naming-conventions)
9. [API Design](#api-design)
10. [Performance Considerations](#performance-considerations)

---

## 🎯 Quick Reference

**For AI Assistants:** Read this section first, then reference details as needed.

```c
// ✅ CORRECT STYLE - Follow this pattern:

#pragma once
// Vehicle Dynamics Engine - Module Name

#include "math/math_base.h"
#include "math/vec3.h"

//-------------------------
// Configuration
//-------------------------

#define MODULE_CONSTANT ((vde_real)1.0)

//-------------------------
// Types
//-------------------------

typedef struct ModuleState {
    vde_real value;
    vde_vec3 position;
} ModuleState;

//-------------------------
// API Functions
//-------------------------

VDE_API ModuleState* module_create(vde_real param);
VDE_API void module_destroy(ModuleState* state);
VDE_API int module_update(ModuleState* state, vde_real dt);
```

---

## 🔧 Language & Build

### Language Standard
- **C11** (ISO/IEC 9899:2011)
- Use C11 features: `<stdbool.h>`, `<stdint.h>`, `restrict` keyword
- Prepare for future GPU compatibility (CUDA/OpenCL)

### Compiler Compatibility
- MSVC 2019+ (Windows)
- GCC 7+ (Linux)
- Clang 6+ (macOS)

---

## 📄 Header Files

### Header Guards
**ALWAYS use `#pragma once`** - Never use traditional include guards.

```c
// ✅ CORRECT
#pragma once
// Vehicle Dynamics Engine - Module Name

// ❌ WRONG - Do NOT use
#ifndef MODULE_H
#define MODULE_H
...
#endif
```

**Rationale:** Cleaner, faster compilation, impossible to have naming conflicts.

### Header Structure Template

```c
#pragma once
// Vehicle Dynamics Engine - [Module Name]

//-------------------------
// Includes
//-------------------------

#include "math/math_base.h"  // Always include for vde_real, constants

//-------------------------
// Configuration (optional)
//-------------------------

#define MODULE_CONFIG_VALUE ((vde_real)1.0)

//-------------------------
// Types
//-------------------------

typedef struct ModuleName {
    vde_real data;
} ModuleName;

//-------------------------
// API Functions
//-------------------------

VDE_API ModuleName* module_create(void);
VDE_API void module_destroy(ModuleName* m);
```

### Include Order
1. System headers (`<stdio.h>`, `<math.h>`)
2. Project math headers (`math/`)
3. Project module headers
4. Current module's header (in .c files)

---

## 🔢 Type System

### Floating-Point Types

**ALWAYS use `vde_real`** for all floating-point calculations.

```c
// ✅ CORRECT
vde_real position = (vde_real)10.0;
vde_real velocity = (vde_real)5.5;
vde_real force = mass * acceleration;

// ❌ WRONG - Never use bare double/float
double position = 10.0;  // NO!
float velocity = 5.5f;   // NO!
```

**Rationale:** 
- Allows global precision switching (double ↔ float)
- Prepares for GPU deployment
- Better numerical stability control
- Defined in `math/math_base.h` as:
  ```c
  #ifndef VDE_REAL
  #define VDE_REAL double  // Default: double precision
  #endif
  typedef VDE_REAL vde_real;
  ```

### Mathematical Types

**Use math library types for spatial calculations:**

```c
// ✅ CORRECT - Use math library types
vde_vec3 position = vde_vec3_make(1.0, 2.0, 3.0);
vde_quat orientation = vde_quat_identity();
vde_mat3 rotation = vde_quat_to_mat3(&orientation);
vde_frame body_frame = vde_frame_make(&orientation, &position);

// ❌ WRONG - Don't create your own vector types
struct MyVector { double x, y, z; };  // NO! Use vde_vec3
```

### Integer Types

Use `<stdint.h>` fixed-width types when size matters:

```c
int32_t index;      // Known 32-bit integer
uint64_t timestamp; // Known 64-bit unsigned
size_t count;       // For sizes/counts (platform-dependent)
```

### Boolean Type

```c
#include <stdbool.h>

bool is_valid = true;
if (is_valid) { ... }
```

---

## 📁 Code Organization

### File Structure

#### Header (.h) Files
```c
#pragma once
// Vehicle Dynamics Engine - [Module]

// Includes
// Configuration
// Types
// API Functions
```

#### Implementation (.c) Files
```c
#include "module/module.h"

//-------------------------
// Internal State (if needed)
//-------------------------

struct ModuleInternal {
    // Private data
};

//-------------------------
// Internal Functions (if needed)
//-------------------------

static void internal_helper(void) {
    // ...
}

//-------------------------
// Lifecycle
//-------------------------

ModuleType* module_create(void) {
    // ...
}

void module_destroy(ModuleType* m) {
    // ...
}

//-------------------------
// Core Functionality
//-------------------------

int module_update(ModuleType* m, vde_real dt) {
    // ...
}

//-------------------------
// Utility Functions
//-------------------------

void module_get_state(ModuleType* m, StateType* out) {
    // ...
}
```

### Section Headers

Use consistent section dividers:

```c
//-------------------------
// Section Name
//-------------------------
```

**Common sections:**
- Configuration
- Types
- API Functions
- Internal State
- Internal Functions
- Lifecycle
- Core Functionality
- Control
- Dynamics
- Telemetry
- Utility Functions
- State Export

---

## 🛡️ Defensive Programming

### Null Pointer Checks

**ALWAYS validate pointers** at API boundaries:

```c
// ✅ CORRECT
void module_update(Module* m, vde_real dt) {
    if (!m) return;  // Guard against null
    if (dt <= (vde_real)0.0) return;  // Validate inputs
    
    // Safe to use m here
    m->value += dt;
}

// ❌ WRONG - No validation
void module_update(Module* m, vde_real dt) {
    m->value += dt;  // Crash if m is NULL!
}
```

### Allocation Checks

```c
// ✅ CORRECT
Module* module_create(void) {
    Module* m = (Module*)malloc(sizeof(Module));
    if (!m) return NULL;  // Check allocation
    
    m->data = (Data*)malloc(sizeof(Data));
    if (!m->data) {  // Check nested allocation
        free(m);
        return NULL;
    }
    
    return m;
}
```

### Numerical Stability

```c
// ✅ CORRECT - Check for valid numbers
void module_integrate(Module* m, vde_real dt) {
    if (!m) return;
    
    m->velocity += m->acceleration * dt;
    
    // Check for NaN/Inf
    if (!vde_isfinite(m->velocity)) {
        // Reset to safe state
        m->velocity = (vde_real)0.0;
        m->acceleration = (vde_real)0.0;
    }
}
```

### Input Validation & Clamping

```c
// ✅ CORRECT - Clamp inputs to valid ranges
void vehicle_set_throttle(Vehicle* v, vde_real throttle) {
    if (!v) return;
    
    // Clamp to [0, 1]
    v->throttle = vde_clamp(throttle, (vde_real)0.0, (vde_real)1.0);
}
```

### Division by Zero Protection

```c
// ✅ CORRECT
vde_real safe_divide(vde_real numerator, vde_real denominator) {
    if (vde_abs(denominator) < VDE_EPS) {
        return (vde_real)0.0;  // Or other safe default
    }
    return numerator / denominator;
}
```

---

## 📝 Documentation

### File Headers

```c
#pragma once
// Vehicle Dynamics Engine - Module Name
//
// Brief description of what this module does and its purpose
// in the overall simulation.
```

### Function Comments

```c
/* 
 * Brief description of what the function does
 * 
 * Parameters:
 *   param1 - Description
 *   param2 - Description
 * 
 * Returns:
 *   Description of return value, or NULL/error codes
 */
VDE_API ReturnType function_name(ParamType1 param1, ParamType2 param2);
```

### Inline Comments

```c
// Use single-line comments for brief explanations
vde_real force = mass * acceleration;  // F = ma

/* Use multi-line comments for longer explanations
 * that span several lines or need more detail
 */
```

### Constants Documentation

```c
// Mathematical constants
#define VDE_PI        ((vde_real)3.14159265358979323846)  // π
#define VDE_DEG2RAD   (VDE_PI / (vde_real)180.0)         // Degrees to radians
```

---

## 🏷️ Naming Conventions

### Prefix System

All public API uses `vde_` prefix:

```c
vde_vec3      // Type
vde_vec3_add  // Function
VDE_PI        // Constant macro
VDE_API       // API export macro
```

### Case Conventions

```c
// Types: lowercase with underscores
typedef struct vde_vec3 { ... } vde_vec3;
typedef struct Vehicle { ... } Vehicle;

// Functions: lowercase_with_underscores
vde_vec3 vde_vec3_make(vde_real x, vde_real y, vde_real z);
void vehicle_update(Vehicle* v, vde_real dt);

// Constants/Macros: UPPERCASE
#define VDE_PI ((vde_real)3.14159265358979323846)
#define MODULE_MAX_SIZE 1024

// Variables: lowercase_with_underscores
vde_real timestep = (vde_real)0.02;
Vehicle* my_vehicle = vehicle_create();
```

### Module Namespacing

Group related functions with common prefix:

```c
// ✅ GOOD - Clear module hierarchy
vde_vec3_add()
vde_vec3_sub()
vde_vec3_normalize()

vehicle_create()
vehicle_destroy()
vehicle_update()

tire_calculate_forces()
tire_update_slip()
```

---

## 🔌 API Design

### Export Macros

```c
// In header:
#ifndef VDE_API
#  define VDE_API
#endif

VDE_API Module* module_create(void);
VDE_API void module_destroy(Module* m);
```

For Windows DLL support:
```c
#ifdef _WIN32
    #ifdef BUILDING_DLL
        #define VDE_API __declspec(dllexport)
    #else
        #define VDE_API __declspec(dllimport)
    #endif
#else
    #define VDE_API
#endif
```

### C++ Compatibility

```c
#ifdef __cplusplus
extern "C" {
#endif

// C declarations here

#ifdef __cplusplus
}
#endif
```

### Opaque Types Pattern

```c
// In header:
typedef struct Module Module;  // Forward declaration

// In source:
struct Module {
    // Private implementation
    vde_real internal_data;
};
```

### Error Handling

```c
// Return NULL for allocation failures
Module* module_create(void) {
    Module* m = malloc(sizeof(Module));
    if (!m) return NULL;
    return m;
}

// Return status codes for operations
int module_update(Module* m) {
    if (!m) return -1;  // Error
    // ...
    return 0;  // Success
}
```

---

## ⚡ Performance Considerations

### Memory Layout

```c
// ✅ GOOD - Cache-friendly struct layout
typedef struct Vehicle {
    // Hot data first (accessed frequently)
    vde_vec3 position;
    vde_vec3 velocity;
    vde_quat orientation;
    
    // Cold data last (accessed rarely)
    char name[64];
    uint32_t id;
} Vehicle;
```

### Restrict Keyword

Use `VDE_RESTRICT` for pointer aliasing hints:

```c
void vde_vec3_add(vde_vec3* VDE_RESTRICT out, 
                  const vde_vec3* VDE_RESTRICT a, 
                  const vde_vec3* VDE_RESTRICT b) {
    out->x = a->x + b->x;
    out->y = a->y + b->y;
    out->z = a->z + b->z;
}
```

### GPU Compatibility

Mark functions for future GPU use:

```c
// VDE_GPU_QUAL expands to __host__ __device__ for CUDA
VDE_GPU_QUAL vde_real vde_abs(vde_real x) {
    return (x < (vde_real)0) ? -x : x;
}
```

### Inline Small Functions

```c
// In header, for very small functions:
static inline vde_real vde_square(vde_real x) {
    return x * x;
}
```

---

## 📋 Checklist for New Code

Before committing new code, verify:

- [ ] Uses `#pragma once` (not traditional guards)
- [ ] Uses `vde_real` for all floating-point
- [ ] Uses math library types (vde_vec3, vde_quat, etc.)
- [ ] Has section divider comments
- [ ] Includes defensive null checks
- [ ] Validates input parameters
- [ ] Checks allocation results
- [ ] Uses `vde_isfinite()` for numerical stability
- [ ] Follows naming conventions
- [ ] Has basic documentation
- [ ] Properly casts for printf (e.g., `(double)vde_real`)
- [ ] Uses proper API export macros
- [ ] Includes C++ extern "C" guards if needed

---

## 🔄 Keeping This Document Updated

**⚠️ IMPORTANT:** As the codebase evolves, update this document!

When you:
- Add new patterns or conventions
- Discover better practices
- Change existing standards

**Update this file and note the change date at the top.**

---

## 📚 Additional References

- **AI_CONTEXT.md** - Quick reference for AI assistants
- **copilotPlan.txt** - Project structure and expansion plan
- **README.md** - Project overview and build instructions

---

**Questions or Improvements?** Update this document and discuss with the team.
