#!/bin/bash

# -----------------------------------------------------------------------------
# Script : setup_project.sh
# Description : Ce script configure et lance un conteneur Docker avec support 
# graphique (X11) et accès au GPU via NVIDIA Docker.
# -----------------------------------------------------------------------------

# 1. Autorise le conteneur Docker (root) à accéder à l'affichage graphique X11
# -----------------------------------------------------------------------------
# La commande `xhost local:root` permet d'autoriser l'utilisateur root, utilisé 
# par le conteneur, à accéder au serveur X11 de l'hôte. 
# Cela est indispensable pour afficher les interfaces graphiques générées 
# dans le conteneur sur l'écran de l'hôte.
xhost local:root

# 2. Définit la variable pour le fichier d'authentification X11
# -----------------------------------------------------------------------------
# La variable `XAUTH` stocke le chemin vers un fichier d'authentification X11.
# Ce fichier est utilisé pour sécuriser les connexions au serveur X11.
XAUTH=/tmp/.docker.xauth

# 3. Crée le fichier XAUTH s'il n'existe pas
# -----------------------------------------------------------------------------
# Si le fichier d'authentification n'existe pas encore, on le crée et on y 
# ajoute des informations d'authentification en utilisant `xauth`.
if [ ! -f "$XAUTH" ]; then
    touch "$XAUTH"
    xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge -
fi

# 4. Lance le conteneur Docker
# -----------------------------------------------------------------------------
# La commande suivante lance un conteneur Docker avec les configurations 
# nécessaires pour ROS 2, support graphique et accès GPU. Voici une explication 
# détaillée de chaque option :
docker run -it \
    --name=ros2_sdc_container \                    # Nomme le conteneur "ros2_sdc_container"
    --env="DISPLAY=$DISPLAY" \                     # Passe la variable DISPLAY pour l'affichage X11
    --env="QT_X11_NO_MITSHM=1" \                   # Résout les problèmes graphiques avec Qt (désactive MIT-SHM)
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \  # Monte le socket Unix X11 pour la communication graphique
    --env="XAUTHORITY=$XAUTH" \                    # Définit l'emplacement du fichier d'authentification X11
    --volume="$XAUTH:$XAUTH" \                     # Monte le fichier XAUTH dans le conteneur
    --net=host \                                   # Utilise le réseau de l'hôte (nécessaire pour ROS et X11)
    --privileged \                                 # Accorde des privilèges élevés au conteneur
    --runtime=nvidia \                             # Spécifie le runtime NVIDIA pour le support GPU
    docker_name \                                  # Remplacez par l'image Docker que vous souhaitez exécuter
    bash                                           # Lance une session Bash dans le conteneur

# 5. Message de fin
# -----------------------------------------------------------------------------
# Affiche un message pour indiquer que le script s'est exécuté correctement.
echo "Done."
