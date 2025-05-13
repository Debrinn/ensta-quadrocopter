#include "velocity_controller.h"
#include <stdio.h>
#include <stdlib.h>
#include "helpers.h"

void acceleration_ref(float * v_ref, float * v, float * a_ref, float ** K, int dim_vect, int autre_dim_mat){
    float * sous = malloc(sizeof(float) * dim_vect);
    vec_substract(v_ref,v,sous, dim_vect);
    mult_mat_vec(a_ref,K,sous,autre_dim_mat,dim_vect);

}