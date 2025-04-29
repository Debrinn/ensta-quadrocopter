#include <stdio.h>
#include <stdlib.h>
#include "helpers.h"

// Fonction qui multiplie les matrices
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

// Transposée d'une matrice
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

void free_mat(float **A, int rows) {
    for (int i = 0; i < rows; i++) {
        free(A[i]); // Libère chaque ligne
    }
    free(A); // Puis libère le tableau de pointeurs
}

float norm(float **A,int rows,int cols){
    float S=0;
    for(int i=0;i<rows;i++){
        for(int j=0;j<cols;j++){        
            S+=(A[i][j])*(A[i][j]);
        }
    }
    return S;

}


//fonction déterminant

float determinant3x3(float A[3][3]) {
    return A[0][0]*(A[1][1]* A[2][2]-A[1][2]*A[2][1]) -
           A[0][1]*(A[1][0] *A[2][2]-A[1][2]*A[2][0])+
           A[0][2]*(A[1][0] * A[2][1] -A[1][1]*A[2][0]);
}


//fonction inverse


int invert_3x3(float A[3][3], float a_inv[3][3]){
    if (determinant3x3(A) == 0){
        return 0;
    }
    else{

        float COM[3][3]; //création de la comatrice
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

                a_inv[i][j] = TCOM[j][i] / detA;

            }
        }
    }
    return 1;
}

//print une matrice 3x3

void print_matrix( float A[3][3]){
    for (int i = 0 ; i<3 ; i++){
        for (int j = 0 ; j<3 ; j++){

            printf("%1.f ", A[i][j]);
        }
    printf ("\n");
    }
}