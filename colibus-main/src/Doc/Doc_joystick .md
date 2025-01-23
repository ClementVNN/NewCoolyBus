# Fichier **joystick.yaml** du projet **colibus_simulator**

Le fichier **joystick.yaml** contient des paramètres de configuration pour les nœuds liés au joystick et à la commande du robot via ce périphérique. Ce fichier permet de régler les actions et les comportements en fonction des entrées du joystick.

---

## **Contenu du fichier `joystick.yaml`**

### **`joy_node`**
- Ce bloc contient les paramètres du nœud ROS responsable de la gestion du joystick.
- **`device_id`** : L'ID du périphérique joystick à utiliser (par défaut, `0` correspond au premier périphérique joystick détecté).
- **`deadzone`** : La zone morte (en pourcentage) autour du centre des axes du joystick. Si l'entrée du joystick est dans cette zone, elle sera ignorée.
- **`autorepeat_rate`** : La fréquence en Hz à laquelle les boutons du joystick sont automatiquement répétés si maintenus enfoncés.

### **`teleop_node`**
- Ce bloc contient les paramètres du nœud ROS responsable de la téléopération (télécommande) du robot via le joystick.
  
#### **Paramètres linéaires :**
- **`axis_linear`** : Définit les axes qui contrôlent la vitesse linéaire du robot. Par exemple, `x: 1` indique que l'axe X du joystick contrôle la vitesse linéaire.
- **`scale_linear`** : Définit le facteur de mise à l'échelle de la vitesse linéaire pour un mode normal.
- **`scale_linear_turbo`** : Définit le facteur de mise à l'échelle de la vitesse linéaire pour un mode turbo.

#### **Paramètres angulaires :**
- **`axis_angular`** : Définit les axes qui contrôlent la rotation du robot. Par exemple, `yaw: 0` signifie que l'axe Yaw (rotation autour de l'axe vertical) est contrôlé par un axe du joystick.
- **`scale_angular`** : Définit le facteur de mise à l'échelle pour les mouvements angulaires en mode normal.
- **`scale_angular_turbo`** : Définit le facteur de mise à l'échelle pour les mouvements angulaires en mode turbo.

#### **Paramètres des boutons :**
- **`enable_button`** : Le numéro du bouton à utiliser pour activer la commande du robot via le joystick.
- **`enable_turbo_button`** : Le numéro du bouton à utiliser pour activer le mode turbo, qui augmente la vitesse du robot.
- **`require_enable_button`** : Si cette option est activée (`true`), le bouton d'activation doit être pressé pour permettre toute autre action sur le joystick.

---
