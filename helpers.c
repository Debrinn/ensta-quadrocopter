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