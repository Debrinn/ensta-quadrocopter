#include "../includes/helpers.h"
#include "../includes/calcul_t_ref.h"
void compute_t_ref(float ** K, float ** q, float ** q_ref, float ** t_ref){
    float ** sous = make_mat(3,1);
    //Soustraction
    mat_algebrique(q_ref,q,sous,3,1,-1);

    //Multiplication par K
    mat_mult(K,3,3,sous,1,t_ref);
}