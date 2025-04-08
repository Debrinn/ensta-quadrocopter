#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>


void mat_mult(const float *A, int A_rows, int A_cols, const float *B, int B_cols, float *C);
void mat_transpose(const float *A, int rows, int cols, float *A_T);
bool invert_3x3(const float A[3][3], float a_inv[3][3]); // Check that det(A) > 0 before inverting
void vec_substract(const float *a, const float *b, float *c, int dim);