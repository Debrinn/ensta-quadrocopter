#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>


void mat_mult( float **A, int A_rows, int A_cols,  float **B, int B_cols, float **C);
void mat_transpose( float **A, int rows, int cols, float **A_T);
float determinant3x3(float **A); //pour la fonction invert3x3
int invert_3x3( float **A, float **a_inv); // Check that det(A) > 0 before inverting
void vec_substract( float *a,  float *b, float *c, int dim);
void mat_algebrique( float **A,  float **B, float **C, int rows, int cols, int op); // Sert à soustraire ou additionner deux matrices : op = 1 pour ajout et op = -1 pour soustraire
void copy_mat( float **A, float **B, int rows, int cols);
float **make_mat(int rows, int cols);
void free_mat(float **A, int rows);
float norme_mat(float **A,int rows,int cols);

void mult_mat_vec(float * res, float ** M, float * v, int n, int m);

void vect_to_quat(float ** quat, float ** vect, int ligne);

void quat_mult(float ** quat1, float ** quat2, float ** res);

void inv_quat(float ** q, float ** q_inv);

void print_matrix(float** A, int rows, int columns);

void pdt_vect(float **A, float **B, float **C);

void normalise_vecteur(float ** A);

float signe(float x);

float maxi(float a, float b);

void rotation_matrice_vers_quat_chiaverini(float ** R, float ** q);