#include "helpers.h"
#include "params.h"

void compute_lqr_gains(const float A[STATE_DIM][STATE_DIM], 
                    const float B[STATE_DIM][CTRL_DIM],
                    const float Q[STATE_DIM][STATE_DIM],
                    const float R [CTRL_DIM][CTRL_DIM],
                    float K[CTRL_DIM][CTRL_DIM])
{}


int main(int argc, char ** argv){
    return 0;
}