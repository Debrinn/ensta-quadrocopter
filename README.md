
---

# Projet de Quadrocopter, par Thibault et Matéo

---

## Premières fonctions

In the file `helpers.c` you will first write three functions that you will need for the rest of the project:

* `mat_mult(const float *A, int A_rows, int A_cols, const float *B, int B_cols, float *C)` — multiplies two matrices A and B, and stores the result in matrix C.
* `mat_transpose(const float *A, int rows, int cols, float *A_T)` — transposes matrix A into A\_T.
* `invert_3x3(const float A[3][3], float a_inv[3][3])` — inverts a 3x3 matrix A into a\_inv.
* `vec_substract(const float *a, const float *b, float *c, int dim)` — computes vector subtraction $a - b = c$, with vectors of dimension `dim`.

You can test your functions with basic algebra computations before starting the next section.

---

## Fonctions déjà implémentées dans `helpers.c` / `helpers.h`

* `mat_mult(float **A, int A_rows, int A_cols, float **B, int B_cols, float **C)` — multiplies matrices A and B, stores the result in C.
* `mat_transpose(float **A, int rows, int cols, float **A_T)` — transposes matrix A.
* `determinant3x3(float **A)` — computes the determinant of a 3x3 matrix A.
* `invert_3x3(float **A, float **a_inv)` — inverts a 3x3 matrix A into a\_inv.
* `vec_substract(float *a, float *b, float *c, int dim)` — subtracts vector b from vector a and stores the result in vector c.
* `mat_algebrique(float **A, float **B, float **C, int rows, int cols, int op)` — adds or subtracts two matrices A and B depending on `op` (1 for addition, -1 for subtraction), stores result in C.
* `copy_mat(float **A, float **B, int rows, int cols)` — copies matrix A into matrix B.
* `make_mat(int rows, int cols)` — dynamically allocates memory for a matrix of size (rows x cols).
* `free_mat(float **A, int rows)` — frees dynamically allocated memory for a matrix A.
* `norme_mat(float **A, int rows, int cols)` — computes the squared Frobenius norm of matrix A.

---

## Compute A and B matrices and get the Q and R matrices from the paper

To obtain the A and B matrices, you will need physical parameters for the system of interest.
Later, we will try to implement our controller in the PX4 Firmware and will simulate the **x500 quadcopter**.

Physical parameters:

* Mass:
  `m = 2.0`

* Inertia:

  ```math
  I = \begin{bmatrix}
  0.02166666666666667 & 0.0 & 0.0 \\
  0.0 & 0.02166666666666667 & 0.0 \\
  0.0 & 0.0 & 0.04000000000000001
  \end{bmatrix}
  ```

---

## Solver

Now that you have implemented your helper functions, you will define your system and implement the **LQR algorithm** described in the project's documentation (section 3.2).
You should implement your function in the file:

* `lqr_solver.c`

---

## Comment lancer le code

Pour compiler et lancer le projet, utilisez les commandes suivantes dans le terminal :

```bash
cmake .
make
./lqr_solver
```

---

