#include "math/frames.h"

//-------------------------
// Non-inline Implementations
//-------------------------

vde_mat3 vde_frame_R(const vde_frame* f) {
    return vde_quat_to_mat3(&f->q);
}

vde_vec3 vde_frame_vec_body_to_world(const vde_frame* f,const vde_vec3* vb){ return vde_quat_rotate(&f->q, vb); }
vde_vec3 vde_frame_point_body_to_world(const vde_frame* f,const vde_vec3* Xb){
    vde_vec3 Rvb=vde_quat_rotate(&f->q, Xb); vde_vec3 out; vde_vec3_add(&out,&f->p,&Rvb); return out;
}
vde_vec3 vde_frame_vec_world_to_body(const vde_frame* f,const vde_vec3* vw){
    vde_quat qi=vde_quat_inv(&f->q); return vde_quat_rotate(&qi, vw);
}
vde_vec3 vde_frame_point_world_to_body(const vde_frame* f,const vde_vec3* Xw){
    vde_vec3 t; vde_vec3_sub(&t,Xw,&f->p); vde_quat qi=vde_quat_inv(&f->q); return vde_quat_rotate(&qi,&t);
}

vde_frame vde_frame_compose(const vde_frame* a,const vde_frame* b){
    vde_frame r; r.q=vde_quat_mul(&a->q,&b->q); vde_vec3 Rp=vde_quat_rotate(&a->q,&b->p); vde_vec3_add(&r.p,&a->p,&Rp); return r;
}
vde_frame vde_frame_inverse(const vde_frame* f){
    vde_frame r; r.q=vde_quat_inv(&f->q); vde_vec3 negp={-f->p.x,-f->p.y,-f->p.z}; r.p=vde_quat_rotate(&r.q,&negp); return r;
}

vde_frame vde_frame_from_ypr_pos(vde_real yaw,vde_real pitch,vde_real roll,const vde_vec3* pos_w){
    vde_mat3 R=vde_mat3_from_ypr(yaw,pitch,roll); vde_quat q=vde_quat_from_mat3(&R); return vde_frame_make(&q,pos_w);
}
vde_vec3 vde_frame_to_ypr(const vde_frame* f){
    vde_mat3 R=vde_quat_to_mat3(&f->q); return vde_mat3_to_ypr(&R);
}