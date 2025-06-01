#include "helpers.h"
#include <math.h>







void q_ref_calcul(float ** a_ref, float psi_ref, float ** q_ref){

    // Etape  1: calculer z_B = direction de la poussée (alignée avec a_ref)
    float norme_a_ref = norme_mat(a_ref,3,1);
    float ** z_B = make_mat(3,1);
    z_B[0][0] = (1. /norme_a_ref) * a_ref[0][0];
    z_B[1][0] = (1. /norme_a_ref) * a_ref[1][0];
    z_B[2][0] = (1. /norme_a_ref) * a_ref[2][0];

    //Etape 2 : Construire un vecteur horizontal x_h a partir de psi_ref

    float ** x_h = make_mat(3,1);
    x_h[0][0] = cos(psi_ref);
    x_h[1][0] = sin(psi_ref);
    x_h[2][0] = 0.;

    //Etape 3 : Construire y_B = norme(pdt_vect(z_B, x_h))

    float ** y_B = make_mat(3,1);
    pdt_vect(z_B,x_h,y_B);
    normalise_vecteur(y_B);

    //Etape 4 : Construire x_B = pdt_vect y_B, z_B)

    float ** x_B = make_mat(3,1);
    pdt_vect(y_B,z_B,x_B);
    normalise_vecteur(x_B);

    //Etape 5 : Construire la matrice de rotation R = [x_B, y_B, z_B]
    float ** R = make_mat(3,3);

    R[0][0] = x_B[0][0];
    R[1][0] = x_B[1][0];
    R[2][0] = x_B[2][0];

    R[0][1] = y_B[0][0];
    R[1][1] = y_B[1][0];
    R[2][1] = z_B[2][0];
    
    R[0][2] = z_B[0][0];
    R[1][2] = z_B[1][0];
    R[2][2] = z_B[2][0];

    //Etape 5 : Convertir R en  quaterion q_ref

    rotation_matrice_vers_quat_chiaverini(R,q_ref);







}




