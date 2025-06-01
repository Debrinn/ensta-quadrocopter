#include "../includes/velocity_controller.h"
#include <stdio.h>
#include <stdlib.h>
#include "../includes/helpers.h"

//Cette fonction calcule a_ref
void acceleration_ref(float ** v_ref, float ** v, float ** a_ref, float ** K, int dim_vect, int autre_dim_mat){
    float ** sous = malloc(sizeof(float*) * dim_vect);
    for(int i = 0; i < dim_vect; i++){
        sous[i] = malloc(sizeof(float)*1);
    }
    //Soustraction
    mat_algebrique(v_ref,v,sous,dim_vect,1,-1);

    //Multiplication par K
    mat_mult(K,autre_dim_mat,dim_vect,sous,1,a_ref);
    

}

