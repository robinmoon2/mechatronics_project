from mpu6050 import mpu6050
from gpiozero import DistanceSensor
import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor, servo

# --- INITIALISATION ---
sensor_mpu = mpu6050(0x68)
dist_sensor = DistanceSensor(echo=24, trigger=23, max_distance=2)

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x5f)
pca.frequency = 50

# Objets pour les moteurs
moteur_gauche = motor.DCMotor(pca.channels[15], pca.channels[14])
moteur_droit = motor.DCMotor(pca.channels[12], pca.channels[13])

# --- TES FONCTIONS HABITUELLES ---

def set_angle(ID, angle):
    """ID 0 = Direction, ID 1 = Ultrason"""
    srv = servo.Servo(pca.channels[ID], min_pulse=500, max_pulse=2400)
    srv.angle = angle

def checkdist():
    return dist_sensor.distance * 100

def avancer(vitesse):
    throttle = vitesse / 100.0
    moteur_gauche.throttle = throttle
    moteur_droit.throttle = throttle

def stopper():
    moteur_gauche.throttle = 0
    moteur_droit.throttle = 0

# --- LOGIQUE MPU (GYROSCOPE) ---

def obtenir_angle_yaw(angle_cible):
    """
    Fait tourner le robot jusqu'à ce que le gyroscope atteigne l'angle voulu.
    """
    angle_actuel = 0
    last_time = time.time()

    # On tourne jusqu'à atteindre l'objectif
    while abs(angle_actuel) < abs(angle_cible):
        current_time = time.time()
        dt = current_time - last_time

        # On récupère la vitesse de rotation (gyroscope) sur l'axe Z
        gyro_data = sensor_mpu.get_gyro_data()
        vitesse_z = gyro_data['z'] # En degrés par seconde

        angle_actuel += vitesse_z * dt
        last_time = current_time
        time.sleep(0.01)

# --- MANOEUVRE D'ESQUIVE ---

def eviter_obstacle():
    stopper()
    print("Obstacle ! Analyse...")

    # 1. Scan
    set_angle(1, 45) # Gauche
    time.sleep(0.5)
    g = checkdist()
    set_angle(1, 135) # Droite
    time.sleep(0.5)
    d = checkdist()
    set_angle(1, 90) # Centre

    # 2. Choix direction
    direction = "gauche" if g > d else "droite"
    angle_braquage = 65 if direction == "gauche" else 115
    angle_retour = 115 if direction == "gauche" else 65
    cible = 25 if direction == "gauche" else -25

    print(f"Esquive par la {direction}")

    avancer(-20)
    time.sleep(1.2)

    # Étape A : On braque et on tourne de 30°
    set_angle(0, angle_braquage)
    avancer(15)

    time.sleep(0.4)

    # Étape B : On dépasse l'obstacle
    set_angle(0, angle_retour) # On remet le capteur droit
    time.sleep(0.3)
    set_angle(0, 90)
    time.sleep(0.6)

    # Étape C : On revient vers la trajectoire (-30°)
    set_angle(0, angle_retour)
    time.sleep(0.4)


    # Étape D : On se remet droit
    set_angle(0, angle_braquage)
    time.sleep(0.4)
    set_angle(0, 90)

    print("Esquive terminée.")

# --- MAIN ---

def main():
    try:
        set_angle(0, 90)
        set_angle(1, 90)
        while True:
            dist = checkdist()
            if dist < 30:
                eviter_obstacle()
            else:
                set_angle(0, 90)
                avancer(15)
            time.sleep(0.05)
    except KeyboardInterrupt:
        stopper()
        set_angle(0, 90)

if __name__ == "__main__":
    main()
