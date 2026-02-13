#include "math/vec3.h"

//-------------------------
// Non-inline Implementations
//-------------------------

vde_real vde_vec3_norm(const vde_vec3* a) {
    return (vde_real)sqrt(vde_vec3_norm2(a));
}

void vde_vec3_normalize(vde_vec3* VDE_RESTRICT out, const vde_vec3* VDE_RESTRICT a) {
    vde_real n = vde_vec3_norm(a);
    if (n > VDE_SQRT_EPS) {
        vde_real inv = (vde_real)1.0 / n;
        out->x = a->x * inv;
        out->y = a->y * inv;
        out->z = a->z * inv;
    } else {
        *out = vde_vec3_zero();
    }
}