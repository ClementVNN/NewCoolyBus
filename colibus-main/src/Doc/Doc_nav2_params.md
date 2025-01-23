# Fichier **nav2_params.yaml** du projet **colibus_simulator**

Le fichier **nav2_params.yaml** contient des paramètres de configuration pour le système de navigation du robot dans un environnement donné. Il configure plusieurs modules tels que l'AMCL (Adaptive Monte Carlo Localization), le planificateur de trajectoire, les costmaps locaux et globaux, ainsi que les serveurs associés à la navigation autonome.

---

## **Contenu du fichier `nav2_params.yaml`**

### **`amcl`**
- Paramètres de configuration pour le localisateur AMCL (Adaptive Monte Carlo Localization), utilisé pour localiser le robot dans un environnement cartographié à l'aide d'un laser.

#### **Paramètres :**
- **`use_sim_time`** : Utilise ou non le temps simulé. Ici, il est activé (`True`).
- **`alpha1` à `alpha5`** : Paramètres influençant la précision du modèle de mouvement du robot.
- **`base_frame_id`** : Le cadre de référence pour le robot, ici `base_footprint`.
- **`beam_skip_distance`** et autres paramètres liés au laser : Ces paramètres contrôlent la gestion des données du capteur laser, telles que les seuils de distance et d'erreur pour le beam skipping.
- **`max_particles`** : Le nombre maximal de particules utilisées pour la localisation.
- **`laser_max_range` et `laser_min_range`** : Les plages maximales et minimales du capteur laser.
- **`global_frame_id`** : Le cadre de référence global utilisé pour la cartographie, ici `map`.
  
### **`bt_navigator`**
- Paramètres du BT (Behavior Tree) Navigator, responsable de la planification et de la gestion des actions de navigation du robot.

#### **Paramètres :**
- **`use_sim_time`** : Active le temps simulé pour la navigation.
- **`global_frame`** et **`robot_base_frame`** : Définit les cadres de référence pour la navigation globale et du robot.
- **`plugin_lib_names`** : Liste des plugins utilisés par le BT Navigator pour les différentes actions de navigation, telles que la planification de trajet, la conduite assistée et la gestion des objectifs.

### **`controller_server`**
- Paramètres pour le serveur de contrôleur du robot, définissant la fréquence du contrôleur et les seuils de mouvement.

#### **Paramètres :**
- **`controller_frequency`** : Fréquence de mise à jour des contrôleurs.
- **`goal_checker_plugins`** : Liste des plugins utilisés pour vérifier la validité des objectifs.

### **`local_costmap` et `global_costmap`**
- Paramètres pour les costmaps locales et globales, utilisées pour la planification du chemin et l'évitement des obstacles.

#### **Paramètres :**
- **`update_frequency`** et **`publish_frequency`** : Fréquences de mise à jour et de publication des cartes de coût.
- **`plugins`** : Liste des couches utilisées dans la costmap, telles que la couche d'inflation et la couche d'obstacle.
  
### **`map_server`**
- Paramètres pour le serveur de carte, qui charge et sert la carte utilisée par le robot pour la localisation et la navigation.

#### **Paramètres :**
- **`yaml_filename`** : Chemin vers le fichier YAML contenant la carte.

### **`planner_server`**
- Paramètres pour le serveur de planification de trajectoire, qui détermine le meilleur chemin à suivre pour le robot en fonction de l'état de la carte.

#### **Paramètres :**
- **`planner_plugins`** : Liste des plugins utilisés pour la planification du chemin.
  
### **`smoother_server`**
- Paramètres pour le serveur de lissage, utilisé pour améliorer la trajectoire planifiée en rendant le mouvement du robot plus fluide.

#### **Paramètres :**
- **`smoother_plugins`** : Liste des plugins utilisés pour le lissage de trajectoire.

### **`behavior_server`**
- Paramètres pour le serveur de comportements, responsable de la gestion des actions comme tourner, se déplacer, et attendre sur place.

#### **Paramètres :**
- **`behavior_plugins`** : Liste des plugins comportementaux, tels que "spin", "backup" (reculer), et "drive_on_heading" (conduite sur direction).
  
### **`robot_state_publisher`**
- Paramètres pour le robot state publisher, qui publie l'état du robot.

### **`waypoint_follower`**
- Paramètres pour le suiveur de waypoints, utilisé pour guider le robot d'un waypoint à un autre.

### **`velocity_smoother`**
- Paramètres pour l'ajustement de la vitesse, permettant d'ajuster les vitesses linéaires et angulaires du robot afin de les rendre plus fluides et sûres.

---

