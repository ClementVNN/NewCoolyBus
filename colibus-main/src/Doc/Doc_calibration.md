# Fichier **calibration.yaml** du projet **colibus_simulator**

Le fichier **calibration.yaml** contient des paramètres de calibration pour ajuster la position et l'orientation de différents capteurs par rapport au robot. Chaque section définit la transformation entre le cadre de référence du robot (`base_link`) et les différents capteurs.

---

## **Contenu du fichier `calibration.yaml`**

### **`base_link2camera`**
- Ce bloc définit la transformation entre le cadre de référence du robot (`base_link`) et la caméra.
- Il inclut :
  - **x** : la distance en mètres entre le robot et la caméra sur l'axe X.
  - **y** : la distance sur l'axe Y.
  - **z** : la distance sur l'axe Z.
  - **roll** : l'orientation de la caméra autour de l'axe X.
  - **pitch** : l'orientation autour de l'axe Y.
  - **yaw** : l'orientation autour de l'axe Z.

### **`base_link2velodyne`**
- Ce bloc définit la transformation entre le cadre de référence du robot et le LiDAR Velodyne.
- Il inclut les mêmes paramètres que pour la caméra (distance sur les axes X, Y, Z, et les rotations sur les axes X, Y, Z).

### **`base_link2imu`**
- Ce bloc définit la transformation entre le cadre de référence du robot et l'IMU (Unité de Mesure Inertielle).
- Il inclut également les distances et orientations spécifiques à l'IMU par rapport au robot.

---

