# Fichier **my_controllers.yaml** du projet **colibus_simulator**

Le fichier **my_controllers.yaml** contient des paramètres de configuration pour les contrôleurs du robot. Il définit le taux de mise à jour des contrôleurs et les configurations spécifiques pour les différents types de contrôleurs utilisés dans le projet.

---

## **Contenu du fichier `my_controllers.yaml`**

### **`controller_manager`**
- Ce bloc contient les paramètres pour le gestionnaire de contrôleurs, qui orchestre les différents contrôleurs utilisés par le robot.
  
#### **Paramètres :**
- **`update_rate`** : Le taux de mise à jour des contrôleurs en Hertz (Hz). Il définit la fréquence à laquelle les contrôleurs doivent être mis à jour. Par défaut, ici, la fréquence est de 30 Hz.
- **`diff_cont`** : Définit le contrôleur de type `diff_drive_controller/DiffDriveController` pour un robot à roues différentielles.
- **`joint_broad`** : Définit le contrôleur `joint_state_broadcaster/JointStateBroadcaster` qui est responsable de la diffusion des états des joints du robot.

### **`diff_cont`**
- Ce bloc contient des paramètres spécifiques au contrôleur de conduite différentielle du robot, responsable du contrôle des moteurs des roues.
  
#### **Paramètres :**
- **`publish_rate`** : Le taux de publication des commandes de vitesse du contrôleur, en Hertz (Hz). Ici, le contrôleur publie à 50 Hz.
- **`base_frame_id`** : Définit le cadre de référence du robot. Par défaut, il est réglé sur `base_link`, représentant le cadre principal du robot.
- **`left_wheel_names`** : Liste des noms des roues gauches utilisées par le contrôleur. Ici, la roue gauche est définie par `left_wheel_joint`.
- **`right_wheel_names`** : Liste des noms des roues droites utilisées par le contrôleur. Ici, la roue droite est définie par `right_wheel_joint`.
- **`wheel_separation`** : La séparation entre les roues gauche et droite, en mètres. Cette valeur est utilisée pour déterminer le rayon de braquage du robot.
- **`wheel_radius`** : Le rayon des roues, en mètres, nécessaire pour calculer la vitesse linéaire du robot en fonction des rotations des roues.
- **`use_stamped_vel`** : Indique si le contrôleur utilise des commandes de vitesse estampillées (c'est-à-dire avec un timestamp) ou non. Ici, il est réglé sur `false`, ce qui signifie que les vitesses ne sont pas estampillées.
