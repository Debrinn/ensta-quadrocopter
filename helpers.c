# include "helpers.h"


//Fonction qui multiplie les matrices

void mat_mult(const float **A, int A_rows, int A_cols, const float **B, int B_cols, float **C){
    //nombre de colonnes de A est égal au nombre de lignes de B
    // On suppose que les matrices sont de bonne taille

    for(int i = 0; i < A_rows ; i++){
        for(int j = 0 ; j <  B_cols; j ++){
            float res = 0.;
            for (int k = 0; k < A_cols; k++){
                res = res + A[i][k]* B[k][j]; 
            }
            C[i][j] = res ;
        }
    }
}