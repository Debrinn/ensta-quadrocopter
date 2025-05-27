# Projet de Quadrocopter, par Thibault et Matéo




##  Premières fonctions
In the file helpers.c you will first write three fucntions that you will need for the rest of the project:
- ```mat_mult(const float *A, int A_rows, int A_cols, const float *B, int B_cols, float *C)``` where A and B are the matrices to multiply and C is the multiplication resulting matrix.
- ```mat_transpose(const float *A, int rows, int cols, float *A_T)``` transposing matrix A.
- ```invert_3x3(const float A[3][3], float a_inv[3][3])``` invertin matrix A.
- ```vec_substract(const float *a, const float *b, float *c, int dim)```, does $a - b = c$ with $a,b,c \in \mathbb{R}^{dim}$.
You can test your functions with basic algebra computations before starting the next section.

En tout, 9 fonctions ont déjà été ajoutées afin d'aider à réaliser le code.
Elles se trouvent dans le fichier helpers.c (code) et helpers.h (prototypes).

void mat_mult( float **A, int A_rows, int A_cols,  float **B, int B_cols, float **C);
MULTIPLICATION DE DEUX MATRICES

void mat_transpose( float **A, int rows, int cols, float **A )
TRANSPOSEE D'UNE MATRICE A rows LIGNE ET cols COLONNES.

float determinant3x3(float **A);
DETERMINANT D'UNE MATRICE 3X3.

int invert_3x3( float **A, float **a_inv); 
INVERSION D'UNE MATRICE 3X3.

void vec_substract( float *a,  float *b, float *c, int dim);
SOUSTRACTION DE DEUX VECTEURS DE DIMENSION dim.

void mat_algebrique( float **A,  float **B, float **C, int rows, int cols, int op);
Sert à soustraire ou additionner deux matrices : op = 1 pour ajout et op = -1 pour soustraire

void copy_mat( float **A, float **B, int rows, int cols);
COPIE D'UNE MATRICE.

float **make_mat(int rows, int cols);
ALLOCATION DYNAMIQUE D'UNE MATRICE.

void free_mat(float **A, int rows);
FREE DE L'ALLOCATION DYNAMIQUE D'UNE MATRICE.

float norme_mat(float **A,int rows,int cols);
IMPLEMENTATION D'UNE NORME DE FROBENIUS AU CARRE POUR LES MATRICES.


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

## Comment lancer le code 
Pour lancer le projet, il faut :
```cmake .```
et ensuite
```make```
Enfin, le solveur se lancera grâce au code :
```./lqr_solver```

