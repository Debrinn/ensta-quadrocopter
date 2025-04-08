# C Project: Quadcopter UAV velocity control based on a LQR attitude controller 

##  $1^{st}$ steps helper functions
In the file helpers.c you will first write three fucntions that you will need for the rest of the project:
- ```mat_mult(const float *A, int A_rows, int A_cols, const float *B, int B_cols, float *C)``` where A and B are the matrices to multiply and C is the multiplication resulting matrix.
- ```mat_transpose(const float *A, int rows, int cols, float *A_T)``` transposing matrix A.
- ```invert_3x3(const float A[3][3], float a_inv[3][3])``` invertin matrix A.
- ```vec_substract(const float *a, const float *b, float *c, int dim)```, does $a - b = c$ with $a,b,c \in \mathbb{R}^{dim}$.
You can test your functions with basic algebra computations before starting the next section.

## Compute A and B matrices and get the Q and R matrices from the paper.

To obtain the A and B matrices you will need physical parameters for the system of interest. Later we will try to imoplement out controller in the PX4 Frirmware and will simulate the x500 quadcopter drone. Its mass is $m=2.0$ and is inertia is given by:
```math
I = \begin{bmatrix}0.02166666666666667 & 0.0 & 0.0 \\
0.0 & 0.02166666666666667 & 0.0 \\
0.0 & 0.0 & 0.04000000000000001\end{bmatrix}
```
## Solver
Now that you have implemented you helper functions, you will have to define your system and implement the algorithm provided in the project's description sheet in section 3.2.
You can implement your function in lqr_solver.c

## Running the solver
To build and run your project you can use ```cmake .``` then ```make``` to build your code. Then you can you your solver with ```./lqr_solver```