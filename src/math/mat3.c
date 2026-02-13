#include "math/mat3.h"

//-------------------------
// Non-inline Implementations
//-------------------------

void vde_mat3_add(vde_mat3* VDE_RESTRICT out, const vde_mat3* VDE_RESTRICT A, const vde_mat3* VDE_RESTRICT B) {
    out->m00=A->m00+B->m00; out->m01=A->m01+B->m01; out->m02=A->m02+B->m02;
    out->m10=A->m10+B->m10; out->m11=A->m11+B->m11; out->m12=A->m12+B->m12;
    out->m20=A->m20+B->m20; out->m21=A->m21+B->m21; out->m22=A->m22+B->m22;
}
void vde_mat3_sub(vde_mat3* VDE_RESTRICT out,const vde_mat3* VDE_RESTRICT A,const vde_mat3* VDE_RESTRICT B){
    out->m00=A->m00-B->m00; out->m01=A->m01-B->m01; out->m02=A->m02-B->m02;
    out->m10=A->m10-B->m10; out->m11=A->m11-B->m11; out->m12=A->m12-B->m12;
    out->m20=A->m20-B->m20; out->m21=A->m21-B->m21; out->m22=A->m22-B->m22;
}
void vde_mat3_scale(vde_mat3* VDE_RESTRICT out,const vde_mat3* VDE_RESTRICT A,vde_real s){
    out->m00=A->m00*s; out->m01=A->m01*s; out->m02=A->m02*s;
    out->m10=A->m10*s; out->m11=A->m11*s; out->m12=A->m12*s;
    out->m20=A->m20*s; out->m21=A->m21*s; out->m22=A->m22*s;
}
void vde_mat3_mul(vde_mat3* VDE_RESTRICT out,const vde_mat3* VDE_RESTRICT A,const vde_mat3* VDE_RESTRICT B){
    vde_mat3 R;
    R.m00=A->m00*B->m00 + A->m01*B->m10 + A->m02*B->m20;
    R.m01=A->m00*B->m01 + A->m01*B->m11 + A->m02*B->m21;
    R.m02=A->m00*B->m02 + A->m01*B->m12 + A->m02*B->m22;

    R.m10=A->m10*B->m00 + A->m11*B->m10 + A->m12*B->m20;
    R.m11=A->m10*B->m01 + A->m11*B->m11 + A->m12*B->m21;
    R.m12=A->m10*B->m02 + A->m11*B->m12 + A->m12*B->m22;

    R.m20=A->m20*B->m00 + A->m21*B->m10 + A->m22*B->m20;
    R.m21=A->m20*B->m01 + A->m21*B->m11 + A->m22*B->m21;
    R.m22=A->m20*B->m02 + A->m21*B->m12 + A->m22*B->m22;
    *out=R;
}
void vde_mat3_vec3(vde_vec3* VDE_RESTRICT out,const vde_mat3* VDE_RESTRICT A,const vde_vec3* VDE_RESTRICT v){
    out->x=A->m00*v->x + A->m01*v->y + A->m02*v->z;
    out->y=A->m10*v->x + A->m11*v->y + A->m12*v->z;
    out->z=A->m20*v->x + A->m21*v->y + A->m22*v->z;
}
vde_real vde_mat3_det(const vde_mat3* A){
    return A->m00*(A->m11*A->m22 - A->m12*A->m21)
         - A->m01*(A->m10*A->m22 - A->m12*A->m20)
         + A->m02*(A->m10*A->m21 - A->m11*A->m20);
}
void vde_mat3_transpose(vde_mat3* VDE_RESTRICT out,const vde_mat3* VDE_RESTRICT A){
    vde_mat3 T={ A->m00, A->m10, A->m20, A->m01, A->m11, A->m21, A->m02, A->m12, A->m22 }; *out=T;
}
bool vde_mat3_inv(vde_mat3* VDE_RESTRICT out,const vde_mat3* VDE_RESTRICT A){
    vde_real D=vde_mat3_det(A); if(vde_abs(D)<=VDE_EPS){ *out=vde_mat3_zero(); return false; }
    vde_real invD=(vde_real)1.0/D;
    vde_mat3 C;
    C.m00=(A->m11*A->m22 - A->m12*A->m21)*invD;
    C.m01=-(A->m10*A->m22 - A->m12*A->m20)*invD;
    C.m02=(A->m10*A->m21 - A->m11*A->m20)*invD;

    C.m10=-(A->m01*A->m22 - A->m02*A->m21)*invD;
    C.m11=(A->m00*A->m22 - A->m02*A->m20)*invD;
    C.m12=-(A->m00*A->m21 - A->m01*A->m20)*invD;

    C.m20=(A->m01*A->m12 - A->m02*A->m11)*invD;
    C.m21=-(A->m00*A->m12 - A->m02*A->m10)*invD;
    C.m22=(A->m00*A->m11 - A->m01*A->m10)*invD;

    *out=C; return true;
}
vde_mat3 vde_mat3_skew(const vde_vec3* v){
    return vde_mat3_make( 0, -v->z,  v->y,  v->z, 0, -v->x, -v->y, v->x, 0 );
}
vde_mat3 vde_mat3_from_axis_angle(const vde_vec3* axis, vde_real angle){
    vde_vec3 a_n; vde_vec3_normalize(&a_n, axis);
    vde_real c=(vde_real)cos(angle), s=(vde_real)sin(angle), C=(vde_real)1.0 - c;
    vde_real x=a_n.x, y=a_n.y, z=a_n.z;
    return vde_mat3_make( c + x*x*C, x*y*C - z*s, x*z*C + y*s,
                          y*x*C + z*s, c + y*y*C, y*z*C - x*s,
                          z*x*C - y*s, z*y*C + x*s, c + z*z*C );
}
vde_mat3 vde_mat3_from_ypr(vde_real yaw, vde_real pitch, vde_real roll){
    vde_real cy=(vde_real)cos(yaw), sy=(vde_real)sin(yaw);
    vde_real cp=(vde_real)cos(pitch), sp=(vde_real)sin(pitch);
    vde_real cr=(vde_real)cos(roll),  sr=(vde_real)sin(roll);
    return vde_mat3_make( cy*cp,            cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr,
                          sy*cp,            sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr,
                         -sp,               cp*sr,             cp*cr );
}
vde_vec3 vde_mat3_to_ypr(const vde_mat3* R){
    vde_real pitch = -(vde_real)asin(vde_clamp(R->m20, (vde_real)-1.0, (vde_real)1.0));
    vde_real cp = (vde_real)cos(pitch);
    vde_real yaw, roll;
    if (vde_abs(cp) > VDE_SQRT_EPS) { yaw=(vde_real)atan2(R->m10,R->m00); roll=(vde_real)atan2(R->m21,R->m22); }
    else { yaw=0; roll=(vde_real)atan2(-R->m01,R->m11); }
    return vde_vec3_make(yaw,pitch,roll);
}
vde_mat3 vde_mat3_orthonormalize(const vde_mat3* A){
    vde_vec3 c0={A->m00,A->m10,A->m20}, c1={A->m01,A->m11,A->m21}, c2={A->m02,A->m12,A->m22};
    vde_vec3 c0n; vde_vec3_normalize(&c0n,&c0);
    vde_real d=vde_vec3_dot(&c1,&c0n);
    vde_vec3 t; vde_vec3_madd(&t,&c1,&c0n,-d);
    vde_vec3 c1n; vde_vec3_normalize(&c1n,&t);
    vde_vec3 c2n; vde_vec3_cross(&c2n,&c0n,&c1n);
    return vde_mat3_make(c0n.x,c1n.x,c2n.x, c0n.y,c1n.y,c2n.y, c0n.z,c1n.z,c2n.z);
}