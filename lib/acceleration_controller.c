#include "../includes/acceleration_controller.h"
#include "../includes/helpers.h"

//void transpose_q(q, q_et)


void trouver_T_ref(float * a_ref, float * g, float * q, float m, float** T_ref){
    float ** a_quad = make_mat(4,1);
    vect_to_quat(a_quad,a_ref,0);
    float ** g_quad = make_mat(4,1);
    vect_to_quat(g_quad,g,0);
    float ** a_quad_g_quad = make_mat(4,1);
    mat_algebrique(a_quad,g_quad,a_quad_g_quad,4,1,-1);
    float ** q_x_moins = make_mat(4,1);
    quat_mult(q,a_quad_g_quad,q_x_moins);
    
    float ** inv = make_mat(4,1);
    inv_quat(q,inv);
    quat_mult(q_x_moins,inv,T_ref);
    T_ref[0][0] = m * T_ref[0][0];
    T_ref[1][0] = m * T_ref[1][0];
    T_ref[2][0] = m * T_ref[2][0];
    T_ref[3][0] = m * T_ref[3][0];


    free_mat(a_quad,4);

    free_mat(g_quad,4);

    free_mat(a_quad_g_quad,4);

    free_mat(q_x_moins,4);

    free_mat(inv,4);
  

    
}