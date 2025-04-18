#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>


void mat_mult( float **A, int A_rows, int A_cols,  float **B, int B_cols, float **C);
void mat_transpose( float **A, int rows, int cols, float **A_T);
bool invert_3x3( float A[3][3], float a_inv[3][3]); // Check that det(A) > 0 before inverting
void vec_substract( float *a,  float *b, float *c, int dim);
void mat_algebrique( float **A,  float **B, float **C, int rows, int cols, int op); // Sert à soustraire ou additionner deux matrices : op = 1 pour ajout et op = -1 pour soustraire
void copy_mat( float **A, float **B, int rows, int cols);
float **make_mat(int rows, int cols);
void free_mat(float **A, int rows);
float norm(float **A,int rows,int cols);