#include "math/quat.h"

//-------------------------
// Non-inline Implementations
//-------------------------

vde_quat vde_quat_from_axis_angle(const vde_vec3* axis, vde_real angle) {
    vde_vec3 a_n;
    vde_vec3_normalize(&a_n, axis);
    vde_real half = angle * (vde_real)0.5;
    vde_real s = (vde_real)sin(half);
    return vde_quat_make((vde_real)cos(half), a_n.x * s, a_n.y * s, a_n.z * s);
}

vde_real vde_quat_norm(const vde_quat* q) {
    return (vde_real)sqrt(vde_quat_norm2(q));
}

vde_quat vde_quat_normalize(const vde_quat* q) {
    vde_real n = vde_quat_norm(q);
    if (n <= VDE_SQRT_EPS) return vde_quat_identity();
    vde_real inv = (vde_real)1.0 / n;
    return vde_quat_make(q->w * inv, q->x * inv, q->y * inv, q->z * inv);
}

vde_quat vde_quat_inv(const vde_quat* q) {
    vde_real n2 = vde_quat_norm2(q);
    if (n2 <= VDE_EPS) return vde_quat_identity();
    vde_real inv = (vde_real)1.0 / n2;
    vde_quat qc = vde_quat_conj(q);
    return vde_quat_make(qc.w * inv, qc.x * inv, qc.y * inv, qc.z * inv);
}

vde_quat vde_quat_mul(const vde_quat* a,const vde_quat* b){
    vde_quat r;
    r.w = a->w*b->w - a->x*b->x - a->y*b->y - a->z*b->z;
    r.x = a->w*b->x + a->x*b->w + a->y*b->z - a->z*b->y;
    r.y = a->w*b->y - a->x*b->z + a->y*b->w + a->z*b->x;
    r.z = a->w*b->z + a->x*b->y - a->y*b->x + a->z*b->w;
    return r;
}

vde_vec3 vde_quat_rotate(const vde_quat* q_raw,const vde_vec3* v){
    vde_quat q=vde_quat_normalize(q_raw);
    vde_vec3 qv={q.x,q.y,q.z};
    vde_vec3 t; vde_vec3_cross(&t,&qv,v); vde_vec3_scale(&t,&t,(vde_real)2.0);
    vde_vec3 wxt; vde_vec3_cross(&wxt,&qv,&t);
    vde_vec3 out=*v; vde_vec3_madd(&out,&out,&t,q.w); vde_vec3_add(&out,&out,&wxt);
    return out;
}

vde_mat3 vde_quat_to_mat3(const vde_quat* q_raw){
    vde_quat q=vde_quat_normalize(q_raw);
    vde_real xx=q.x*q.x, yy=q.y*q.y, zz=q.z*q.z;
    vde_real xy=q.x*q.y, xz=q.x*q.z, yz=q.y*q.z;
    vde_real wx=q.w*q.x, wy=q.w*q.y, wz=q.w*q.z;
    return vde_mat3_make(
        1 - 2*(yy + zz),  2*(xy - wz),      2*(xz + wy),
        2*(xy + wz),      1 - 2*(xx + zz),  2*(yz - wx),
        2*(xz - wy),      2*(yz + wx),      1 - 2*(xx + yy)
    );
}

vde_quat vde_quat_from_mat3(const vde_mat3* R){
    vde_real tr=R->m00 + R->m11 + R->m22;
    vde_quat q;
    if(tr>(vde_real)0){
        vde_real S=(vde_real)sqrt(tr + (vde_real)1.0) * (vde_real)2.0;
        q.w=(vde_real)0.25*S; q.x=(R->m21 - R->m12)/S; q.y=(R->m02 - R->m20)/S; q.z=(R->m10 - R->m01)/S;
    } else if (R->m00>R->m11 && R->m00>R->m22){
        vde_real S=(vde_real)sqrt((vde_real)1.0 + R->m00 - R->m11 - R->m22) * (vde_real)2.0;
        q.w=(R->m21 - R->m12)/S; q.x=(vde_real)0.25*S; q.y=(R->m01 + R->m10)/S; q.z=(R->m02 + R->m20)/S;
    } else if (R->m11>R->m22){
        vde_real S=(vde_real)sqrt((vde_real)1.0 + R->m11 - R->m00 - R->m22) * (vde_real)2.0;
        q.w=(R->m02 - R->m20)/S; q.x=(R->m01 + R->m10)/S; q.y=(vde_real)0.25*S; q.z=(R->m12 + R->m21)/S;
    } else {
        vde_real S=(vde_real)sqrt((vde_real)1.0 + R->m22 - R->m00 - R->m11) * (vde_real)2.0;
        q.w=(R->m10 - R->m01)/S; q.x=(R->m02 + R->m20)/S; q.y=(R->m12 + R->m21)/S; q.z=(vde_real)0.25*S;
    }
    return vde_quat_normalize(&q);
}

vde_quat vde_quat_slerp(const vde_quat* q0_raw,const vde_quat* q1_raw,vde_real t){
    vde_quat a=vde_quat_normalize(q0_raw);
    vde_quat b=vde_quat_normalize(q1_raw);
    vde_real cosd=a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
    if(cosd<(vde_real)0){ b.w=-b.w; b.x=-b.x; b.y=-b.y; b.z=-b.z; cosd=-cosd; }
    if(cosd>(vde_real)0.9995){
        vde_quat r=vde_quat_make(a.w*(1-t)+b.w*t, a.x*(1-t)+b.x*t, a.y*(1-t)+b.y*t, a.z*(1-t)+b.z*t);
        return vde_quat_normalize(&r);
    }
    vde_real theta=(vde_real)acos(vde_clamp(cosd,(vde_real)-1.0,(vde_real)1.0));
    vde_real s0=(vde_real)sin(((vde_real)1.0 - t)*theta);
    vde_real s1=(vde_real)sin(t*theta);
    vde_real invs=(vde_real)1.0 / (vde_real)sin(theta);
    vde_quat r=vde_quat_make(a.w*(s0*invs)+b.w*(s1*invs), a.x*(s0*invs)+b.x*(s1*invs), a.y*(s0*invs)+b.y*(s1*invs), a.z*(s0*invs)+b.z*(s1*invs));
    return vde_quat_normalize(&r);
}

vde_quat vde_quat_integrate(const vde_quat* q_raw,const vde_vec3* omega_body,vde_real dt){
    vde_real wnorm=vde_vec3_norm(omega_body); if(wnorm<=(vde_real)1e-12) return vde_quat_normalize(q_raw);
    vde_vec3 axis; vde_vec3_normalize(&axis, omega_body);
    vde_quat dq=vde_quat_from_axis_angle(&axis, wnorm*dt);
    vde_quat r=vde_quat_mul(&dq, q_raw);
    return vde_quat_normalize(&r);
}