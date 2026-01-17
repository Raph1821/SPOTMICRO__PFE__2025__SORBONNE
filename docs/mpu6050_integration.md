# Guide d'Intégration du MPU6050 dans Spot Micro

## 📋 Vue d'Ensemble

Ce guide explique comment intégrer un capteur IMU MPU6050 à votre robot Spot Micro qui utilise déjà :
- Raspberry Pi 4 (8GB)
- Carte PCA9685 (contrôle des servos via I2C)

## 🔌 Connexions Matérielles

### Architecture I2C

Les deux périphériques partagent le **même bus I2C** :

```
Raspberry Pi 4
    │
    ├── Bus I2C-1 (GPIO 2/3)
    │   │
    │   ├── PCA9685 (Adresse 0x40) → Contrôle 12 servos
    │   │
    │   └── MPU6050 (Adresse 0x68) → IMU 6DOF
```

### Branchement MPU6050

| Pin MPU6050 | Pin RPi 4 | GPIO | Description |
|-------------|-----------|------|-------------|
| **VCC** | Pin 1 | 3.3V | Alimentation |
| **GND** | Pin 6 | GND | Masse |
| **SCL** | Pin 5 | GPIO 3 (SCL) | Horloge I2C |
| **SDA** | Pin 3 | GPIO 2 (SDA) | Données I2C |
| **XDA** | - | - | Non connecté |
| **XCL** | - | - | Non connecté |
| **AD0** | GND | - | Adresse 0x68 |
| **INT** | - | - | Non connecté (optionnel) |

**⚠️ Notes importantes :**
- Utilisez l'alimentation **3.3V** (pas 5V)
- Les pins SDA/SCL ont déjà des résistances pull-up sur le RPi
- Le PCA9685 et MPU6050 ont des adresses différentes → pas de conflit

## 🛠️ Installation Logicielle

### 1. Installer les dépendances système

```bash
# Sur la Raspberry Pi
sudo apt-get update
sudo apt-get install -y python3-smbus i2c-tools

# Installer la bibliothèque Python pour I2C
sudo pip3 install smbus2
```

### 2. Activer l'interface I2C

```bash
sudo raspi-config
# Naviguer vers : Interface Options → I2C → Enable → Yes
# Redémarrer
sudo reboot
```

### 3. Vérifier les périphériques I2C

```bash
# Après redémarrage
sudo i2cdetect -y 1
```

**Sortie attendue :**
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --  ← PCA9685
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --  ← MPU6050
70: -- -- -- -- -- -- -- --
```

### 4. Compiler le package ROS

```bash
cd ~/catkin_ws  # Ou votre workspace ROS
catkin_make

# Ou si vous utilisez catkin build
catkin build mpu6050_imu

# Sourcer
source devel/setup.bash
```

## 🚀 Utilisation

### Test basique du MPU6050

```bash
# Terminal 1 : Lancer roscore
roscore

# Terminal 2 : Lancer le nœud IMU
roslaunch mpu6050_imu mpu6050.launch

# Terminal 3 : Vérifier les données
rostopic echo /imu/data_raw
```

### Lancer Spot Micro avec l'IMU

```bash
# Lancez tout le système (servos + IMU)
roslaunch spot_micro_launch spot_micro_with_imu.launch
```

## 📊 Topics ROS Disponibles

| Topic | Type | Fréquence | Description |
|-------|------|-----------|-------------|
| `/imu/data_raw` | `sensor_msgs/Imu` | 50 Hz | Accélération et vitesse angulaire |
| `/imu/temperature` | `sensor_msgs/Temperature` | 50 Hz | Température du capteur |

### Structure du message IMU

```python
# sensor_msgs/Imu
header:
  stamp: rospy.Time.now()
  frame_id: "imu_link"

# Orientation (non disponible sans magnétomètre)
orientation: [x, y, z, w]
orientation_covariance: [-1, 0, 0, 0, -1, 0, 0, 0, -1]  # Unknown

# Vitesse angulaire (rad/s)
angular_velocity:
  x: gyro_x  # Roll rate
  y: gyro_y  # Pitch rate
  z: gyro_z  # Yaw rate

# Accélération linéaire (m/s²)
linear_acceleration:
  x: accel_x
  y: accel_y
  z: accel_z  # Devrait être ~9.81 au repos
```

## 🔧 Utilisation Avancée

### 1. Calculer l'orientation (Roll/Pitch)

Créez un nœud pour calculer les angles à partir de l'accélération :

```python
import math

def calculate_orientation(accel_x, accel_y, accel_z):
    """Calcule roll et pitch depuis l'accéléromètre"""
    roll = math.atan2(accel_y, accel_z)
    pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
    return roll, pitch
```

### 2. Filtrer les données (Filtre Complémentaire)

Combinez gyroscope et accéléromètre pour une meilleure estimation :

```python
# Filtre complémentaire simple
alpha = 0.98  # Coefficient (0.95-0.99)
dt = 0.02     # Période d'échantillonnage (50 Hz)

# Angle depuis gyroscope (intégration)
angle_gyro = prev_angle + gyro_rate * dt

# Angle depuis accéléromètre
angle_accel = calculate_orientation(accel_x, accel_y, accel_z)

# Fusion
angle_filtered = alpha * angle_gyro + (1 - alpha) * angle_accel
```

### 3. Intégrer avec le contrôle de mouvement

Modifiez le code de contrôle pour utiliser l'IMU :

```cpp
// Dans spot_micro_motion_cmd.cpp
// Ajouter un subscriber pour l'IMU
ros::Subscriber imu_sub_ = nh.subscribe("/imu/data_raw", 1, 
                                        &SpotMicroMotionCmd::imuCallback, this);

void SpotMicroMotionCmd::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Récupérer l'orientation
    float roll = calculateRoll(msg->linear_acceleration);
    float pitch = calculatePitch(msg->linear_acceleration);
    
    // Utiliser pour la stabilisation
    // adjustBodyAngle(roll, pitch);
}
```

## 🎯 Applications Possibles

1. **Stabilisation active** : Compenser l'inclinaison du terrain
2. **Détection de chute** : Arrêter le robot si inclinaison > seuil
3. **Odométrie améliorée** : Combiner avec les commandes de vitesse
4. **Cartographie SLAM** : Fournir l'orientation pour le LIDAR
5. **Détection de collision** : Détecter les impacts via accélération

## 🐛 Dépannage

### Problème : "IOError: [Errno 121] Remote I/O error"

**Cause** : MPU6050 non détecté sur le bus I2C

**Solutions** :
1. Vérifier les connexions physiques
2. Vérifier l'alimentation (3.3V)
3. Tester avec `i2cdetect -y 1`
4. Vérifier que I2C est activé

### Problème : Conflit d'adresse I2C

**Cause** : Deux périphériques avec la même adresse

**Solution** : Modifier l'adresse du MPU6050
```python
# Connecter AD0 à VCC au lieu de GND
# L'adresse devient 0x69
self.address = 0x69
```

### Problème : Données bruitées

**Solutions** :
1. Ajouter un filtre passe-bas logiciel
2. Réduire la fréquence d'échantillonnage
3. Calibrer les offsets (bias)
4. Utiliser un filtre de Kalman

### Problème : Permission refusée

```bash
# Solution temporaire
sudo chmod 666 /dev/i2c-1

# Solution permanente
sudo usermod -a -G i2c $USER
# Puis se déconnecter/reconnecter
```

## 📈 Calibration

### Calibration des biais (offsets)

1. Posez le robot sur une surface **parfaitement plane**
2. Lancez le nœud et enregistrez 1000 échantillons
3. Calculez la moyenne des accélérations et vitesses angulaires
4. Soustrayez ces valeurs dans le code

```python
# Exemple de calibration
accel_x_offset = 0.05  # m/s²
accel_y_offset = -0.03
accel_z_offset = 9.81  # Devrait être proche de g

gyro_x_offset = 0.01   # rad/s
gyro_y_offset = -0.02
gyro_z_offset = 0.005

# Dans le code
accel_x_calibrated = accel_x - accel_x_offset
```

## 📚 Références

- [Datasheet MPU6050](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [ROS sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)
- [Raspberry Pi I2C](https://www.raspberrypi.org/documentation/hardware/raspberrypi/)

## 🎓 Prochaines Étapes

1. ✅ Installer et tester le MPU6050
2. ⬜ Calibrer les biais
3. ⬜ Implémenter un filtre complémentaire
4. ⬜ Intégrer dans le contrôle de mouvement
5. ⬜ Ajouter la détection de chute
6. ⬜ Améliorer l'odométrie
