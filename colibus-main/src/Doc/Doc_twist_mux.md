# Fichier **twist_mux.yaml** du projet **colibus_simulator**

Le fichier **twist_mux.yaml** configure le multiplexeur de commandes de vitesse (TwistMux). Ce système permet de gérer plusieurs sources de commandes de mouvement (par exemple, le joystick, la navigation, et le suivi de trajectoire) et de déterminer quelle commande doit être exécutée à un moment donné, en fonction de la priorité et du délai.

---

## **Contenu du fichier `twist_mux.yaml`**

### **`twist_mux`**
- Paramètres de configuration pour le multiplexeur des commandes de vitesse du robot.

#### **Paramètres :**
- **`topics`** : Définit les différents topics qui peuvent être utilisés pour envoyer des commandes de mouvement au robot.
  - **`navigation`** : Topic pour les commandes de mouvement liées à la navigation (`cmd_vel`).
    - **`timeout`** : Temps d'attente maximal pour ce topic avant de considérer la commande comme expirée.
    - **`priority`** : Priorité de ce topic. Plus la valeur est élevée, plus la priorité est grande.
  - **`tracker`** : Topic pour les commandes de mouvement liées au suivi de trajectoire (`cmd_vel_tracker`).
    - **`timeout`** : Temps d'attente maximal pour ce topic avant expiration.
    - **`priority`** : Priorité du topic.
  - **`joystick`** : Topic pour les commandes de mouvement envoyées par le joystick (`cmd_vel_joy`).
    - **`timeout`** : Temps d'attente pour la commande envoyée par le joystick.
    - **`priority`** : Priorité de la commande envoyée par le joystick.

---

