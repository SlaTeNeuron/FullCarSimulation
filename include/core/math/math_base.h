
#pragma once
// Vehicle Dynamics Engine - Math Base (C11)

#include <math.h>
#include <float.h>
#include <stdint.h>
#include <stdbool.h>

//-------------------------
// Configuration
//-------------------------

// Precision: default double; can be set to float for faster performance at the cost of accuracy and stability (not recommended for this sim, but may be useful for other applications of the math library)
#ifndef VDE_REAL
#define VDE_REAL double
#endif
typedef VDE_REAL vde_real;

// Export / Visibility control (adjust for your build system if needed)
#ifndef VDE_API
#  define VDE_API
#endif

// Restrict macro: good for CPU auto-vectorization, maps to CUDA too.
#ifndef VDE_RESTRICT
#  if defined(__CUDACC__) || defined(__clang__) || defined(__GNUC__)
#    define VDE_RESTRICT __restrict__
#  else
#    define VDE_RESTRICT restrict
#  endif
#endif

// Optional GPU host/device compatibility wrapper
#ifndef VDE_GPU_QUAL
#  ifdef __CUDACC__
#    define VDE_GPU_QUAL __host__ __device__
#  else
#    define VDE_GPU_QUAL
#  endif
#endif

// Constants
#define VDE_PI        ((vde_real)3.14159265358979323846264338327950288)
#define VDE_HALF_PI   ((vde_real)1.57079632679489661923132169163975144)
#define VDE_TWO_PI    ((vde_real)6.28318530717958647692528676655900576)
#define VDE_DEG2RAD   (VDE_PI / (vde_real)180.0)
#define VDE_RAD2DEG   ((vde_real)180.0 / VDE_PI)
#define VDE_EPS       ((vde_real)1e-9)
#define VDE_SQRT_EPS  ((vde_real)1e-6)

//-------------------------
// Inline Helper Functions
//-------------------------

// Absolute value
static inline vde_real vde_abs(vde_real x) {
    return (x < (vde_real)0) ? -x : x;
}

// Clamp value to range [lo, hi]
static inline vde_real vde_clamp(vde_real v, vde_real lo, vde_real hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

// Check if value is finite (not NaN or Inf)
static inline bool vde_isfinite(vde_real x) {
    return isfinite(x);  // C11 macro handles float/double/long double
}

// Approximate equality with relative tolerance
static inline bool vde_approx(vde_real a, vde_real b, vde_real tol) {
    vde_real aa = vde_abs(a);
    vde_real bb = vde_abs(b);
    vde_real ref = (aa > bb ? aa : bb);
    if (ref < (vde_real)1.0) ref = (vde_real)1.0;
    return vde_abs(a - b) <= tol * ref;
}
// Min/Max functions
static inline vde_real vde_min(vde_real a, vde_real b) {
    return (a < b) ? a : b;
}

static inline vde_real vde_max(vde_real a, vde_real b) {
    return (a > b) ? a : b;
}

// Sign function: returns -1, 0, or 1
static inline vde_real vde_sign(vde_real x) {
    return (x > (vde_real)0) ? (vde_real)1 : ((x < (vde_real)0) ? (vde_real)-1 : (vde_real)0);
}

// Square function (common in physics calculations)
static inline vde_real vde_square(vde_real x) {
    return x * x;
}

// Linear interpolation: lerp(a, b, t) = a + t*(b - a)
static inline vde_real vde_lerp(vde_real a, vde_real b, vde_real t) {
    return a + t * (b - a);
}

//-------------------------
// Standard Math Function Wrappers
// (for type safety and future GPU compatibility)
//-------------------------

static inline vde_real vde_sqrt(vde_real x) {
    return (vde_real)sqrt(x);
}

static inline vde_real vde_sin(vde_real x) {
    return (vde_real)sin(x);
}

static inline vde_real vde_cos(vde_real x) {
    return (vde_real)cos(x);
}

static inline vde_real vde_tan(vde_real x) {
    return (vde_real)tan(x);
}

static inline vde_real vde_atan2(vde_real y, vde_real x) {
    return (vde_real)atan2(y, x);
}

static inline vde_real vde_asin(vde_real x) {
    return (vde_real)asin(x);
}

static inline vde_real vde_acos(vde_real x) {
    return (vde_real)acos(x);
}

static inline vde_real vde_atan(vde_real x) {
    return (vde_real)atan(x);
}

static inline vde_real vde_pow(vde_real x, vde_real y) {
    return (vde_real)pow(x, y);
}

static inline vde_real vde_exp(vde_real x) {
    return (vde_real)exp(x);
}

static inline vde_real vde_log(vde_real x) {
    return (vde_real)log(x);
}

static inline vde_real vde_floor(vde_real x) {
    return (vde_real)floor(x);
}

static inline vde_real vde_ceil(vde_real x) {
    return (vde_real)ceil(x);
}