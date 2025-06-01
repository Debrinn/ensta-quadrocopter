#include <stdio.h>
#include <stdlib.h>
#include "../includes/helpers.h"
#include <math.h>
// Fonction qui multiplie deux matrices
void mat_mult( float **A, int A_rows, int A_cols,  float **B, int B_cols, float **C) {
    for (int i = 0; i < A_rows; i++) {
        for (int j = 0; j < B_cols; j++) {
            float res = 0.;
            for (int k = 0; k < A_cols; k++) {
                res += A[i][k] * B[k][j];
            }
            C[i][j] = res;
        }
    }
}

// Transposée d'une matrice dans la matrice A_T (QUI EST DEJA DE BONNE TAILLE)
void mat_transpose( float **A, int rows, int cols, float **A_T) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            A_T[j][i] = A[i][j];
        }
    }
}

// Soustraction de vecteurs
void vec_substract( float *a,  float *b, float *c, int dim) {
    for (int i = 0; i < dim; i++) {
        c[i] = a[i] - b[i];
    }
}

// Addition/soustraction de matrices
void mat_algebrique( float **A,  float **B, float **C, int rows, int cols, int op) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            C[i][j] = A[i][j] + op * B[i][j];
        }
    }
}

// Copie de matrice
void copy_mat( float **A, float **B, int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            B[i][j] = A[i][j];
        }
    }
}

// Allocation dynamique de matrice 2D
float **make_mat(int rows, int cols) {
    float **A = malloc(rows * sizeof(float *));
    for (int i = 0; i < rows; i++) {
        A[i] = malloc(cols * sizeof(float));
    }
    return A;
}

//Libère la matrice

void free_mat(float **A, int rows) {
    for (int i = 0; i < rows; i++) {
        free(A[i]); // Libère chaque ligne
    }
    free(A); // Puis libère le tableau de pointeurs
}

//Calcul la norme d'une matrice
float norme_mat(float **A,int rows,int cols){
    float S=0;
    for(int i=0;i<rows;i++){
        for(int j=0;j<cols;j++){        
            S+=(A[i][j])*(A[i][j]);
        }
    }
    return S;

}


//fonction déterminant

float determinant3x3(float **A) {
    return A[0][0]*(A[1][1]* A[2][2]-A[1][2]*A[2][1]) -
           A[0][1]*(A[1][0] *A[2][2]-A[1][2]*A[2][0])+
           A[0][2]*(A[1][0] * A[2][1] -A[1][1]*A[2][0]);
}


//fonction inverse


int invert_3x3(float **A, float **a_inv){
    if (determinant3x3(A) == 0){
        return 0;
    }
    else{
        //création de la comatrice
        float **COM = malloc(sizeof(float*)*3); 
        for (int i = 0; i <3; i++){
            float *l= malloc(sizeof(float)*3);
            COM[i] = l;
        }
        //Remplissage de la commatrice
        COM[0][0] = A[1][1] * A[2][2] - A[1][2] * A[2][1];
        COM[0][1] = -1*(A[1][0] * A[2][2] - A[1][2] * A[2][0]);
        COM[0][2] = A[1][0] * A[2][1] - A[1][1] * A[2][0];
        COM[1][0] = -1*(A[0][1] * A[2][2] - A[0][2] * A[2][1]);
        COM[1][1] = A[0][0] * A[2][2] - A[0][2] * A[2][0];
        COM[1][2] = -1*(A[0][0] * A[2][1] - A[0][1] * A[2][0]);
        COM[2][0] = A[0][1] * A[1][2] - A[0][2] * A[1][1];
        COM[2][1] = -1*(A[0][0] * A[1][2] - A[0][2] * A[1][0]);
        COM[2][2] = A[0][0] * A[1][1] - A[0][1] * A[1][0];

        float detA = determinant3x3(A); //det A

        // inverse comatrice
        //on a Tcom(A)/det(A)=A^-1 pour det(A)!=0

        for ( int i = 0 ; i<3 ; i++ ){
            for( int j = 0 ; j<3 ; j++ ){

                a_inv[i][j] = COM[j][i] / detA;

            }
        }
    }
    return 1;
}



//Fonction de multiplication d'une matrice par un vecteur

void mult_mat_vec(float * res, float ** M, float * v, int n, int m){
    //le res est de taille n, v de taille m
    for(int i = 0; i < n; i++){
        float somme = 0. ;
        for(int j = 0; j < m; j ++){
            somme = somme + M[i][j] * v[i];

        }
        res[i] = somme;
    }
}

//Affichage matrice 
void print_matrix(float** A, int rows, int columns) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < columns; j++) {
            printf("%8.3f ", A[i][j]);
        }
        printf("\n");
    }
}

// -----FONCTIONS liées aux quaternions ------

void vect_to_quat(float ** quat, float ** vect, int ligne){
    quat[0][0] = 0;
    for (int i = 0; i < 3; i ++){
        if (ligne == 1){
            quat[i+1][0] = vect[0][i];
        }else{
            quat[i+1][0] = vect[i][0];

        }
    }
}

void quat_mult(float ** quat1, float ** quat2, float ** res){
    res[0][0] = quat1[0][0]* quat2[0][0] - quat1[1][0]*quat2[1][0] - quat1[2][0]*quat2[2][0] - quat1[3][0]*quat2[3][0];
    res[1][0] = quat1[0][0]*quat2[1][0] + quat1[1][0]*quat2[0][0] + quat1[2][0]*quat2[3][0] - quat1[3][0]* quat2[2][0];
    res[2][0] = quat1[0][0]*quat2[2][0] - quat1[1][0]*quat2[3][0] + quat1[2][0]*quat2[0][0] + quat1[3][0]* quat2[1][0];
    res[3][0] = quat1[0][0]*quat2[3][0] + quat1[1][0]*quat2[2][0] - quat1[2][0]*quat2[1][0] + quat1[3][0]* quat2[0][0];
}

void inv_quat(float ** q, float ** q_inv){
    q_inv[0][0] = q[0][0];
    q_inv[1][0] = -1 * q[1][0];
    q_inv[2][0] = -1 * q[2][0];
    q_inv[3][0] = -1 * q[3][0];
}





// ----- FONCTIONS UTILES POUR calculer q_ref -----


void pdt_vect(float **A, float **B, float **C){
    C[0][0] = A[1][0] * B[2][0] - A[2][0] * B[1][0];
    C[1][0] = A[2][0] * B[0][0] - A[0][0] * B[2][0];
    C[2][0] = A[0][0] * B[1][0] - A[1][0] * B[0][0];
}

void normalise_vecteur(float ** A){
    float n = norme_mat(A,3,1);
    A[0][0] = (1. / n) * A[0][0];
    A[1][0] = (1. / n) * A[1][0];
    A[2][0] = (1. / n) * A[2][0];

}


// Signe de x, retourne +1.0 si x > 0, -1.0 si x < 0, 0 si x = 0
float signe(float x) {
    if (x > 0) return 1.0;
    else if (x < 0) return -1.0;
    else return 0.0;
}

// Convertit une matrice de rotation (3x3) en quaternion [q0, q1, q2, q3]
// Méthode Chiaverini–Siciliano

float maxi(float a, float b){
    if(a>b){
        return a;
    }else{
        return b;
    }
}

void rotation_matrice_vers_quat_chiaverini(float ** R, float ** q) {
    float r11 = R[0][0], r12 = R[0][1], r13 = R[0][2];
    float r21 = R[1][0], r22 = R[1][1], r23 = R[1][2];
    float r31 = R[2][0], r32 = R[2][1], r33 = R[2][2];

    q[0][0] = 0.5 * sqrt(maxi(0.0, 1 + r11 + r22 + r33));
    q[1][0] = 0.5 * sqrt(maxi(0.0, 1 + r11 - r22 - r33)) * signe(r32 - r23);
    q[2][0] = 0.5 * sqrt(maxi(0.0, 1 - r11 + r22 - r33)) * signe(r13 - r31);
    q[3][0] = 0.5 * sqrt(maxi(0.0, 1 - r11 - r22 + r33)) * signe(r21 - r12);
}
