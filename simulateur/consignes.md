# Simulateur de test simplifié

Afin de tester vos algorithmes, vous pourrez utiliser le simulateur fourni dans ce dossier.

Une fois votre code écrit, définissez une fonction dans votre exécutable principal (appelez-le `controller.c` ; vous indiquerez son chemin d’accès à la ligne 10 du fichier `run_3d.py`) appelée `compute_control`.

Cette fonction sera déclarée comme suit :

```c
void compute_control(const double *state,
                     const double *cmd,
                     double *output)
```

•	state est un tableau contenant la vitesse, l’attitude et la vitesse de rotation du drone. \
•	cmd contient les consignes de vitesse pour le drone (on ignorera la consigne de yaw — mettez-la à 0 dans vos codes).\
•	output est un pointeur de taille 4 contenant : tau_x, tau_y, tau_z et thrust.

Le référentiel inertiel est droit et orienté avec l’axe z vers le haut.

Installez l’environnement Python de préférence avec Conda. Les prérequis sont donnés dans le fichier environment.yml.
Lancez le simulateur avec la commande suivante :

```bash
python run_3d.py
```