#pragma once
#include "math_base.h"
#include "vec3.h"

// 3x3 matrix, column-major
typedef struct {
    // | m00 m01 m02 |
    // | m10 m11 m12 |
    // | m20 m21 m22 |
    vde_real m00, m01, m02;
    vde_real m10, m11, m12;
    vde_real m20, m21, m22;
} vde_mat3;

//-------------------------
// Inline Constructors
//-------------------------

static inline vde_mat3 vde_mat3_identity(void) {
    vde_mat3 M = {
        (vde_real)1.0, (vde_real)0.0, (vde_real)0.0,
        (vde_real)0.0, (vde_real)1.0, (vde_real)0.0,
        (vde_real)0.0, (vde_real)0.0, (vde_real)1.0
    };
    return M;
}

static inline vde_mat3 vde_mat3_zero(void) {
    vde_mat3 M = {
        (vde_real)0.0, (vde_real)0.0, (vde_real)0.0,
        (vde_real)0.0, (vde_real)0.0, (vde_real)0.0,
        (vde_real)0.0, (vde_real)0.0, (vde_real)0.0
    };
    return M;
}

static inline vde_mat3 vde_mat3_make(vde_real m00, vde_real m01, vde_real m02,
                                     vde_real m10, vde_real m11, vde_real m12,
                                     vde_real m20, vde_real m21, vde_real m22) {
    vde_mat3 M = {m00, m01, m02, m10, m11, m12, m20, m21, m22};
    return M;
}

//-------------------------
// Non-inline Functions (complex operations)
//-------------------------

VDE_GPU_QUAL void     vde_mat3_add (vde_mat3* VDE_RESTRICT out, const vde_mat3* VDE_RESTRICT A, const vde_mat3* VDE_RESTRICT B);
VDE_GPU_QUAL void     vde_mat3_sub (vde_mat3* VDE_RESTRICT out, const vde_mat3* VDE_RESTRICT A, const vde_mat3* VDE_RESTRICT B);
VDE_GPU_QUAL void     vde_mat3_scale(vde_mat3* VDE_RESTRICT out, const vde_mat3* VDE_RESTRICT A, vde_real s);
VDE_GPU_QUAL void     vde_mat3_mul (vde_mat3* VDE_RESTRICT out, const vde_mat3* VDE_RESTRICT A, const vde_mat3* VDE_RESTRICT B);
VDE_GPU_QUAL void     vde_mat3_vec3(vde_vec3* VDE_RESTRICT out, const vde_mat3* VDE_RESTRICT A, const vde_vec3* VDE_RESTRICT v);

VDE_GPU_QUAL vde_real vde_mat3_det (const vde_mat3* A);
VDE_GPU_QUAL void     vde_mat3_transpose(vde_mat3* VDE_RESTRICT out, const vde_mat3* VDE_RESTRICT A);
VDE_GPU_QUAL bool     vde_mat3_inv(vde_mat3* VDE_RESTRICT out, const vde_mat3* VDE_RESTRICT A);

VDE_GPU_QUAL vde_mat3 vde_mat3_skew(const vde_vec3* v);
VDE_GPU_QUAL vde_mat3 vde_mat3_from_axis_angle(const vde_vec3* axis, vde_real angle);
VDE_GPU_QUAL vde_mat3 vde_mat3_from_ypr(vde_real yaw, vde_real pitch, vde_real roll);
VDE_GPU_QUAL vde_vec3 vde_mat3_to_ypr(const vde_mat3* R);
VDE_GPU_QUAL vde_mat3 vde_mat3_orthonormalize(const vde_mat3* A);