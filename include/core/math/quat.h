#pragma once
#include "math_base.h"
#include "vec3.h"
#include "mat3.h"

typedef struct { vde_real w, x, y, z; } vde_quat;

//-------------------------
// Inline Constructors & Simple Operations
//-------------------------

static inline vde_quat vde_quat_make(vde_real w, vde_real x, vde_real y, vde_real z) {
    vde_quat q = {w, x, y, z};
    return q;
}

static inline vde_quat vde_quat_identity(void) {
    return vde_quat_make((vde_real)1.0, (vde_real)0.0, (vde_real)0.0, (vde_real)0.0);
}

static inline vde_real vde_quat_norm2(const vde_quat* q) {
    return q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
}

static inline vde_quat vde_quat_conj(const vde_quat* q) {
    return vde_quat_make(q->w, -q->x, -q->y, -q->z);
}

//-------------------------
// Non-inline Functions (require sqrt, trig, or complex logic)
//-------------------------

VDE_GPU_QUAL vde_quat vde_quat_from_axis_angle(const vde_vec3* axis, vde_real angle);
VDE_GPU_QUAL vde_real vde_quat_norm (const vde_quat* q);
VDE_GPU_QUAL vde_quat vde_quat_normalize(const vde_quat* q);
VDE_GPU_QUAL vde_quat vde_quat_inv(const vde_quat* q);

VDE_GPU_QUAL vde_quat vde_quat_mul(const vde_quat* a, const vde_quat* b);
VDE_GPU_QUAL vde_vec3 vde_quat_rotate(const vde_quat* q, const vde_vec3* v);
VDE_GPU_QUAL vde_mat3 vde_quat_to_mat3(const vde_quat* q);
VDE_GPU_QUAL vde_quat vde_quat_from_mat3(const vde_mat3* R);

VDE_GPU_QUAL vde_quat vde_quat_slerp(const vde_quat* q0, const vde_quat* q1, vde_real t);
VDE_GPU_QUAL vde_quat vde_quat_integrate(const vde_quat* q, const vde_vec3* omega_body, vde_real dt);