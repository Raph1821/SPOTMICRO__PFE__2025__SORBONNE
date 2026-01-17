# MPU6050 IMU ROS Package

ROS driver pour le capteur IMU MPU6050 (accéléromètre + gyroscope 6DOF) via I2C.

## 📋 Caractéristiques

- **Accéléromètre 3 axes** : ±2g, ±4g, ±8g, ±16g
- **Gyroscope 3 axes** : ±250, ±500, ±1000, ±2000 °/s
- **Température interne**
- Communication **I2C**
- Publishes standard `sensor_msgs/Imu` messages

## 🔌 Connexion Matérielle

### MPU6050 → Raspberry Pi 4

| MPU6050 Pin | Raspberry Pi Pin | Description |
|-------------|------------------|-------------|
| VCC         | Pin 1 (3.3V)     | Alimentation 3.3V |
| GND         | Pin 6 (GND)      | Masse |
| SCL         | Pin 5 (GPIO 3)   | I2C Clock |
| SDA         | Pin 3 (GPIO 2)   | I2C Data |
| AD0         | GND              | Adresse I2C 0x68 (défaut) |

**⚠️ Important :** Le MPU6050 et le PCA9685 partagent le même bus I2C. Assurez-vous qu'ils ont des adresses différentes :
- **PCA9685** : 0x40 (par défaut)
- **MPU6050** : 0x68 (par défaut)

## 📦 Installation

### 1. Installer les dépendances Python

```bash
sudo apt-get install python3-smbus i2c-tools
sudo pip3 install smbus2
```

### 2. Activer I2C sur Raspberry Pi

```bash
sudo raspi-config
# Interface Options → I2C → Enable
```

### 3. Vérifier la connexion I2C

```bash
# Lister les périphériques I2C
sudo i2cdetect -y 1

# Vous devriez voir :
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- --
# ...
# 40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# ...
# 60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
```

## 🚀 Utilisation

### Lancer le nœud IMU

```bash
roslaunch mpu6050_imu mpu6050.launch
```

### Topics publiés

| Topic | Type | Description |
|-------|------|-------------|
| `/imu/data_raw` | `sensor_msgs/Imu` | Données brutes IMU (accel + gyro) |
| `/imu/temperature` | `sensor_msgs/Temperature` | Température interne |

### Visualiser les données

```bash
# Afficher les données IMU
rostopic echo /imu/data_raw

# Visualiser dans RViz
rosrun rviz rviz
# Add → By Topic → /imu/data_raw → Imu
```

## ⚙️ Configuration

Modifier les paramètres dans [launch/mpu6050.launch](launch/mpu6050.launch) :

```xml
<param name="i2c_bus" value="1" />        <!-- Bus I2C (1 pour RPi) -->
<param name="i2c_address" value="0x68" /> <!-- Adresse I2C -->
<param name="frame_id" value="imu_link" /> <!-- Frame ID -->
<param name="rate" value="50" />           <!-- Fréquence (Hz) -->
```

## 🔧 Intégration avec Spot Micro

Pour intégrer l'IMU dans le système Spot Micro, ajoutez dans vos launch files :

```xml
<include file="$(find mpu6050_imu)/launch/mpu6050.launch" />
```

## 📊 Calibration (Optionnelle)

Pour améliorer la précision, calibrez l'IMU :

```bash
# Lancez le nœud
roslaunch mpu6050_imu mpu6050.launch

# Posez le robot sur une surface plane
# Enregistrez les valeurs moyennes pendant 30 secondes
# Utilisez ces valeurs comme offsets dans le code
```

## 🐛 Dépannage

### Erreur "I2C device not found"
- Vérifiez les connexions physiques
- Assurez-vous que I2C est activé : `sudo raspi-config`
- Vérifiez l'adresse : `sudo i2cdetect -y 1`

### Erreur de permission
```bash
sudo chmod 666 /dev/i2c-1
# Ou ajoutez votre utilisateur au groupe i2c
sudo usermod -a -G i2c $USER
```

### Conflit d'adresse I2C
Si plusieurs périphériques partagent la même adresse, changez l'adresse du MPU6050 :
- Connectez le pin AD0 à VCC pour utiliser l'adresse 0x69

## 📝 License

MIT
