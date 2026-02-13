#pragma once
#include "math_base.h"
#include "vec3.h"
#include "mat3.h"
#include "quat.h"

// Rigid transform: orientation (quaternion) + translation (world coords)
typedef struct {
    vde_quat q;  // body -> world
    vde_vec3 p;  // world position
} vde_frame;

//-------------------------
// Inline Constructors
//-------------------------

static inline vde_frame vde_frame_make(const vde_quat* q, const vde_vec3* p) {
    vde_frame f;
    f.q = *q;
    f.p = *p;
    return f;
}

static inline vde_frame vde_frame_identity(void) {
    vde_frame f;
    f.q = vde_quat_identity();
    f.p = vde_vec3_zero();
    return f;
}

//-------------------------
// Non-inline Functions (complex transformations)
//-------------------------

VDE_GPU_QUAL vde_mat3  vde_frame_R(const vde_frame* f);

VDE_GPU_QUAL vde_vec3  vde_frame_vec_body_to_world(const vde_frame* f, const vde_vec3* vb);
VDE_GPU_QUAL vde_vec3  vde_frame_point_body_to_world(const vde_frame* f, const vde_vec3* Xb);
VDE_GPU_QUAL vde_vec3  vde_frame_vec_world_to_body(const vde_frame* f, const vde_vec3* vw);
VDE_GPU_QUAL vde_vec3  vde_frame_point_world_to_body(const vde_frame* f, const vde_vec3* Xw);

VDE_GPU_QUAL vde_frame vde_frame_compose(const vde_frame* a, const vde_frame* b);
VDE_GPU_QUAL vde_frame vde_frame_inverse(const vde_frame* f);

VDE_GPU_QUAL vde_frame vde_frame_from_ypr_pos(vde_real yaw, vde_real pitch, vde_real roll, const vde_vec3* pos_w);
VDE_GPU_QUAL vde_vec3  vde_frame_to_ypr(const vde_frame* f);
