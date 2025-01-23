# Sommaire des fichiers de configuration du projet **colibus_simulator**

Ce document présente un sommaire de tous les fichiers de configuration présents dans le dossier **config** du projet **colibus_simulator**. Chaque fichier contient des paramètres nécessaires à la configuration et au contrôle du robot dans la simulation.

---

## **Liste des fichiers de configuration**

1. **[calibration.yaml](calibration.yaml)**  
   Ce fichier définit les paramètres de calibration des capteurs et des systèmes, incluant les positions relatives entre différentes parties du robot (comme la caméra, LiDAR et IMU) par rapport au châssis du robot.

2. **[joystick.yaml](joystick.yaml)**  
   Ce fichier contient les paramètres pour le noeud du joystick, notamment l'assignation des axes et des boutons du joystick, ainsi que l'échelle de contrôle pour la télécommande du robot.

3. **[my_controllers.yaml](my_controllers.yaml)**  
   Ce fichier configure le gestionnaire de contrôleurs du robot, incluant le contrôleur de conduite différentielle et le gestionnaire d'état des joints du robot, avec les paramètres de base pour la commande des roues.

4. **[nav2_params.yaml](nav2_params.yaml)**  
   Ce fichier configure les paramètres du système de navigation du robot avec Nav2, incluant la configuration du planificateur, des cartes de coûts (locales et globales), des comportements de navigation, et des paramètres d'odométrie et de capteurs.

5. **[twist_mux.yaml](twist_mux.yaml)**  
   Ce fichier configure le multiplexeur de commandes `cmd_vel`, permettant de gérer la priorisation de plusieurs sources de commande de vitesse (par exemple, joystick, suivi de trajectoire, etc.).

6. **[vehicle_info.yaml](vehicle_info.yaml)**  
   Ce fichier contient les informations physiques et géométriques du véhicule, incluant la masse, les dimensions, le rayon de braquage, et l'angle de direction maximal.

---

