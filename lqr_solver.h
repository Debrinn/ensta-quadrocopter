#include "params.h"

void compute_lqr_gains( float A[STATE_DIM][STATE_DIM], 
    float B[STATE_DIM][CTRL_DIM],
    float Q[STATE_DIM][STATE_DIM],
    float R [CTRL_DIM][CTRL_DIM],
   float K[CTRL_DIM][CTRL_DIM]);