#include "helpers.h"
#include "params.h"
#include "lqr_solver.h"

//Cette fonction applique l'algorithme défini dans l'énoncé, qui permet de calculer le gain K
void compute_lqr_gains( float **A, 
                     float **B,
                     float **Q,
                     float **R,
                    float **K)
{

    //Les tailles des matrices :
    // A : STATE_DIM x STATE_DIM
    // B : STATE_DIM x CTRL_DIM
    // Q : STATE_DIM x STATE_DIM
    // R : CTRL_DIM x CTRL_DIM
    // K : CTRL_DIM x CTRL_DIM
    //generation de la matrice PK avec malloc
    float ** pk = make_mat(STATE_DIM,STATE_DIM);
    copy_mat(Q,pk,STATE_DIM,STATE_DIM);
    for (int k = 0; k < MAX_ITER; k++){
        // *** IMPORTANT ***
        // Comme on code en C, et que les fonctions de base entrent dans une matrice les valeurs des transposées, produits, sommes,
        // il nous faut pour chaque transposée, somme, produit... créer une nouvelle matrice vierge pour y insérer les éléments
        float ** pk_plus_1 = make_mat(STATE_DIM,STATE_DIM);
        float ** at_pk_a = make_mat(STATE_DIM,STATE_DIM);
        float ** a_tra = make_mat(STATE_DIM,STATE_DIM);
        float ** pk_a = make_mat(STATE_DIM,STATE_DIM);
        mat_transpose(A,STATE_DIM,STATE_DIM,a_tra);
        mat_mult(pk,STATE_DIM,STATE_DIM,A,STATE_DIM,pk_a);
        mat_mult(a_tra,STATE_DIM,STATE_DIM,pk_a,STATE_DIM,at_pk_a);

        float ** pk_b = make_mat(STATE_DIM,CTRL_DIM);
        float ** b_tra = make_mat(CTRL_DIM,STATE_DIM);
        float **bt_pk_b = make_mat(CTRL_DIM,CTRL_DIM);

        mat_transpose(B,STATE_DIM,CTRL_DIM,b_tra);
        mat_mult(pk,STATE_DIM,STATE_DIM,B, CTRL_DIM ,pk_b);
        mat_mult(b_tra,CTRL_DIM,STATE_DIM,pk_b,CTRL_DIM,bt_pk_b);

        float ** r_plus_bt_pk_b = make_mat(CTRL_DIM,CTRL_DIM);
        mat_algebrique(R,bt_pk_b,r_plus_bt_pk_b,CTRL_DIM,CTRL_DIM, 1);
        float ** inv_r_plus_bt_pk_b = make_mat(CTRL_DIM,CTRL_DIM);
        invert_3x3(r_plus_bt_pk_b,inv_r_plus_bt_pk_b);

        float ** bt_pk_a = make_mat(CTRL_DIM,STATE_DIM);
        mat_mult(b_tra,CTRL_DIM,STATE_DIM,pk_a,STATE_DIM,bt_pk_a);

        float ** at_pk_b = make_mat(STATE_DIM,CTRL_DIM);
        mat_mult(a_tra,STATE_DIM,STATE_DIM,pk_b,CTRL_DIM,at_pk_b);


        float ** inv_r_plus_bt_pk_b_bt_pk_a = make_mat(CTRL_DIM,STATE_DIM);
        mat_mult(inv_r_plus_bt_pk_b,CTRL_DIM,CTRL_DIM,bt_pk_a,STATE_DIM,inv_r_plus_bt_pk_b_bt_pk_a);

        float ** at_pk_b_inv_r_plus_bt_pk_b_bt_pk_a =make_mat(STATE_DIM,STATE_DIM);
        mat_mult(at_pk_b,STATE_DIM,CTRL_DIM,inv_r_plus_bt_pk_b_bt_pk_a,STATE_DIM,at_pk_b_inv_r_plus_bt_pk_b_bt_pk_a);

        float ** le_moins = make_mat(STATE_DIM,STATE_DIM);

        mat_algebrique(at_pk_a,at_pk_b_inv_r_plus_bt_pk_b_bt_pk_a,le_moins,STATE_DIM,STATE_DIM, 1);

        mat_algebrique(Q,le_moins,pk_plus_1,STATE_DIM,STATE_DIM,1);


        /*ON va libérer toutes les matrices*/
        // Libération des matrices
        free_mat(at_pk_a, STATE_DIM);
        free_mat(a_tra, STATE_DIM);
        free_mat(pk_a, STATE_DIM);
        free_mat(pk_b, STATE_DIM);
        free_mat(b_tra, CTRL_DIM);
        free_mat(bt_pk_b, CTRL_DIM);
        free_mat(r_plus_bt_pk_b, CTRL_DIM);
        free_mat(inv_r_plus_bt_pk_b, CTRL_DIM);
        free_mat(bt_pk_a, CTRL_DIM);
        free_mat(at_pk_b, STATE_DIM);
        free_mat(inv_r_plus_bt_pk_b_bt_pk_a, CTRL_DIM);
        free_mat(at_pk_b_inv_r_plus_bt_pk_b_bt_pk_a, STATE_DIM);
        free_mat(le_moins,STATE_DIM);


        /*Fin libération*/



        float ** difference = make_mat(STATE_DIM,STATE_DIM);
        mat_algebrique(pk_plus_1,pk,difference, STATE_DIM,STATE_DIM,-1);
        free_mat(pk, STATE_DIM);
        pk = pk_plus_1;
        if ((norme_mat(difference,STATE_DIM,STATE_DIM)) < TOL){
            break;
        }




    }
    //nous avons trouvé à ce stade un Pk+1 (enregistré pk) tel que la différence Pk+1 - pk est inférieure à l'erreur que l'on s'autorise

    //De nouveau beaucoup de calculs
    float ** at_pk_a = make_mat(STATE_DIM,STATE_DIM);
    float ** a_tra = make_mat(STATE_DIM,STATE_DIM);
    float ** pk_a = make_mat(STATE_DIM,STATE_DIM);
    mat_transpose(A,STATE_DIM,STATE_DIM,a_tra);
    mat_mult(pk,STATE_DIM,STATE_DIM,A,STATE_DIM,pk_a);
    mat_mult(a_tra,STATE_DIM,STATE_DIM,pk_a,STATE_DIM,at_pk_a);


    float ** pk_b = make_mat(STATE_DIM,CTRL_DIM);
    float ** b_tra = make_mat(CTRL_DIM,STATE_DIM);
    float **bt_pk_b = make_mat(CTRL_DIM,CTRL_DIM);

    mat_transpose(B,STATE_DIM,CTRL_DIM,b_tra);
    mat_mult(pk,STATE_DIM,STATE_DIM,B, CTRL_DIM ,pk_b);
    mat_mult(b_tra,CTRL_DIM,STATE_DIM,pk_b,CTRL_DIM,bt_pk_b);

    float ** r_plus_bt_pk_b = make_mat(CTRL_DIM,CTRL_DIM);
    mat_algebrique(R,bt_pk_b,r_plus_bt_pk_b,CTRL_DIM,CTRL_DIM, 1);
    float ** inv_r_plus_bt_pk_b = make_mat(CTRL_DIM,CTRL_DIM);
    invert_3x3(r_plus_bt_pk_b,inv_r_plus_bt_pk_b);

    float ** bt_pk_a = make_mat(CTRL_DIM,STATE_DIM);
    mat_mult(b_tra,CTRL_DIM,STATE_DIM,pk_a,STATE_DIM,bt_pk_a);

    float ** at_pk_b = make_mat(STATE_DIM,CTRL_DIM);
    mat_mult(a_tra,STATE_DIM,STATE_DIM,pk_b,CTRL_DIM,at_pk_b);


    float ** inv_r_plus_bt_pk_b_bt_pk_a = make_mat(CTRL_DIM,STATE_DIM);
    mat_mult(inv_r_plus_bt_pk_b,CTRL_DIM,CTRL_DIM,bt_pk_a,STATE_DIM,inv_r_plus_bt_pk_b_bt_pk_a);

    float ** at_pk_b_inv_r_plus_bt_pk_b_bt_pk_a =make_mat(STATE_DIM,STATE_DIM);
    mat_mult(at_pk_b,STATE_DIM,CTRL_DIM,inv_r_plus_bt_pk_b_bt_pk_a,STATE_DIM,at_pk_b_inv_r_plus_bt_pk_b_bt_pk_a);


    //on enregistre ici le K dans la matrice donnée par la fonction
    mat_algebrique(at_pk_a,at_pk_b_inv_r_plus_bt_pk_b_bt_pk_a, K ,STATE_DIM,STATE_DIM, 1);


    // on libère de nouveau les fonctions dont on a besoin

    free_mat(at_pk_a, STATE_DIM);
    free_mat(a_tra, STATE_DIM);
    free_mat(pk_a, STATE_DIM);
    free_mat(pk_b, STATE_DIM);
    free_mat(b_tra, CTRL_DIM);
    free_mat(bt_pk_b, CTRL_DIM);
    free_mat(r_plus_bt_pk_b, CTRL_DIM);
    free_mat(inv_r_plus_bt_pk_b, CTRL_DIM);
    free_mat(bt_pk_a, CTRL_DIM);
    free_mat(at_pk_b, STATE_DIM);
    free_mat(inv_r_plus_bt_pk_b_bt_pk_a, CTRL_DIM);
    free_mat(at_pk_b_inv_r_plus_bt_pk_b_bt_pk_a, STATE_DIM);

    //on a plus besoin de pk non plus
    free_mat(pk,STATE_DIM);


}


int main(int argc, char ** argv){
    return 0;
}