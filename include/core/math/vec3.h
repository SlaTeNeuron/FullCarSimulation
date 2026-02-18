#pragma once
#include "math_base.h"

// 3D Vector (POD)
typedef struct { vde_real x, y, z; } vde_vec3;

//-------------------------
// Inline Constructors & Simple Operations
//-------------------------

static inline vde_vec3 vde_vec3_make(vde_real x, vde_real y, vde_real z) {
    vde_vec3 v = {x, y, z};
    return v;
}

static inline vde_vec3 vde_vec3_zero(void) {
    return vde_vec3_make((vde_real)0.0, (vde_real)0.0, (vde_real)0.0);
}

static inline vde_vec3 vde_vec3_ex(void) {
    return vde_vec3_make((vde_real)1.0, (vde_real)0.0, (vde_real)0.0);
}

static inline vde_vec3 vde_vec3_ey(void) {
    return vde_vec3_make((vde_real)0.0, (vde_real)1.0, (vde_real)0.0);
}

static inline vde_vec3 vde_vec3_ez(void) {
    return vde_vec3_make((vde_real)0.0, (vde_real)0.0, (vde_real)1.0);
}

static inline void vde_vec3_add(vde_vec3* VDE_RESTRICT out, const vde_vec3* VDE_RESTRICT a, const vde_vec3* VDE_RESTRICT b) {
    out->x = a->x + b->x;
    out->y = a->y + b->y;
    out->z = a->z + b->z;
}

static inline void vde_vec3_sub(vde_vec3* VDE_RESTRICT out, const vde_vec3* VDE_RESTRICT a, const vde_vec3* VDE_RESTRICT b) {
    out->x = a->x - b->x;
    out->y = a->y - b->y;
    out->z = a->z - b->z;
}

static inline void vde_vec3_scale(vde_vec3* VDE_RESTRICT out, const vde_vec3* VDE_RESTRICT a, vde_real s) {
    out->x = a->x * s;
    out->y = a->y * s;
    out->z = a->z * s;
}

static inline vde_real vde_vec3_dot(const vde_vec3* a, const vde_vec3* b) {
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

static inline void vde_vec3_cross(vde_vec3* VDE_RESTRICT out, const vde_vec3* VDE_RESTRICT a, const vde_vec3* VDE_RESTRICT b) {
    out->x = a->y * b->z - a->z * b->y;
    out->y = a->z * b->x - a->x * b->z;
    out->z = a->x * b->y - a->y * b->x;
}

static inline vde_real vde_vec3_norm2(const vde_vec3* a) {
    return vde_vec3_dot(a, a);
}

static inline void vde_vec3_madd(vde_vec3* VDE_RESTRICT out, const vde_vec3* VDE_RESTRICT a, const vde_vec3* VDE_RESTRICT b, vde_real s) {
    out->x = a->x + s * b->x;
    out->y = a->y + s * b->y;
    out->z = a->z + s * b->z;
}

static inline bool vde_vec3_isfinite(const vde_vec3* a) {
    return vde_isfinite(a->x) && vde_isfinite(a->y) && vde_isfinite(a->z);
}

//-------------------------
// Non-inline Functions (require sqrt or complex logic)
//-------------------------

VDE_GPU_QUAL vde_real vde_vec3_norm(const vde_vec3* a);
VDE_GPU_QUAL void     vde_vec3_normalize(vde_vec3* VDE_RESTRICT out, const vde_vec3* VDE_RESTRICT a);