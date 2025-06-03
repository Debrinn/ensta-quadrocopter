
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
## Schéma bloc du projet 

L'objectif du projet est de fournir un système de contrôle pour un quadrocoptere UAV.
Pour cela, on utilisera [la théorie de Monsieur Strivastava](https://arxiv.org/pdf/2404.12261)

![image](https://github.com/user-attachments/assets/92f45e45-f0f4-48ef-8362-cffcae34c76c)

---
## Organisation du code

Le code est organisé en 3 parties.
* `src` : on y trouve controller.c ainsi que le fichier params.h contenant des paramètres obligatoires pour la suite
*  `lib` : on y trouve un fichier `helpers.c` contenant les fonctions génériques utilisées dans notre implémentation des matrices, vecteurs, et quaternions
*  `includes` : on y trouve les .h correspondants aux fichiers de `lib`

## Les fonctions d'aides : `helpers.c`
## Velocity controller : `velocity_controller.c`
Ce bloc fait le calcul suivant
![image](https://github.com/user-attachments/assets/87f7861d-4741-4a8b-9e26-a911b51bf594)

## Acceleration controller : `acceleration_controller.c`
Ce bloc fait le calcul suivant
![image](https://github.com/user-attachments/assets/cbab0bad-54ff-46e9-bead-60d08abbd800)

## T to q_ref : `t_to_q.c`
Ce bloc doit obtenir q_ref, l'attitude voulue du contrôleur.
L'obtention de qref, est plus compliquée. Il faut en fait construire une base orthonormée a partir de a_ref. 
## LQR solver : `lqr_solver.c`
L'algorithme de LQR Solver renvoit une matrice nulle, car la théorie fournie ne fonctionne pas pour passer du modèle continu au modèle discret dans l'algorithme.
Malgré tout voici le schéma de l'algorithme 
![image](https://github.com/user-attachments/assets/b2402841-3b23-40e7-addc-15159b98fcfd)


## Calcul tau ref : `calcul_t_ref.c`
Ce bloc fait le calcul suivant, pour obtenir le couple que l'on donne finalement au robot.
![image](https://github.com/user-attachments/assets/69b1252c-4379-4d7b-87ed-0c5f84bd82bf)

---

## (Partie non supprimée du readme originel // Compute A and B matrices and get the Q and R matrices from the paper

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


```bash
cmake .
make
./lqr_solver
```

---

## Comment lancer le code

Chaque bloc ne contient qu'une fonction, permettant de renvoyer la valeur voulue. Ainsi, pour obtenir si l'on souhaite donc tau ref ainsi que Tref, il suffit d'inclure dans un autre fichier les blocs.h des blocs correspondants.




---

