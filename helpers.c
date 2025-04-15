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

void mat_transpose(const float **A, int rows, int cols, float **A_T)
{
    for(int i=0;i<rows;i++){
        for (int j=0;j<cols;j++){       // On inverse juste les coefficients , sans se soucier de la validité de la taille (on la suppose ok)
            A_T[j][i]=A[i][j];
        }
    }

}


vec_substract(const float *a, const float *b, float *c, int dim){

    for(int i=0; i<dim;i++){
        c[i]=a[i]-b[i]; // Pas besoin de commentaire
    }
}

void copy_mat(const float **A,float **B,int rows, int cols){

    for(int i=0;i<rows;i++){
        for(int j=0;j<cols;j++){        // Recopie une matrice non modifiable en matrice modifiable
            B[i][j]=A[i][j];
        }
    }


}