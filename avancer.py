import time
import math
from adeept_picarpro.front_wheels import FrontWheels
from adeept_picarpro.back_wheels import BackWheels
# On garde vos imports pour les encodeurs via ADS7830
import smbus

# --- CONFIGURATION PHYSIQUE ---
STEER_CENTER = 90  # Angle neutre du servo
TICKS_PER_METER = 500 # À CALIBRER : nombre de ticks pour 1 mètre

class PicarProNavigator:
    def __init__(self):
        self.front_wheels = FrontWheels()
        self.back_wheels = BackWheels()
        self.adc = ADS7830() # Votre classe ADS7830 existante
        self.front_wheels.turn(STEER_CENTER)
        
        # Initialisation des variables d'odométrie
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0 # Orientation en radians

    def get_encoder_ticks(self):
        """
        Lit les ticks actuels via votre méthode ADS7830.
        Note : Pour simplifier, on prend la moyenne des deux encodeurs arrière.
        """
        val_left = self.adc.analog_read(6)  # CH6 selon votre code
        val_right = self.adc.analog_read(6) # Ajustez le canal si différent
        return (val_left + val_right) / 2

    def move_to(self, target_x, target_y):
        """
        Navigue vers une coordonnée relative (x, y)
        """
        # 1. Calcul de la trajectoire
        dx = target_x - self.x
        dy = target_y - self.y
        
        distance = math.sqrt(dx**2 + dy**2)
        target_angle_rad = math.atan2(dx, dy) # Angle vers la cible
        target_angle_deg = math.degrees(target_angle_rad)

        print(f"Trajectoire : {distance:.2f}m à {target_angle_deg:.1f}°")

        # 2. Braquage du train avant
        # On limite le braquage entre 45° et 135° pour protéger le servo
        steer_value = STEER_CENTER + target_angle_deg
        steer_value = max(60, min(120, steer_value))
        self.front_wheels.turn(steer_value)
        time.sleep(0.5)

        # 3. Propulsion contrôlée par encodeurs
        start_ticks = self.get_encoder_ticks()
        target_ticks = distance * TICKS_PER_METER
        
        self.back_wheels.speed = 70 # Vitesse modérée
        self.back_wheels.forward()

        # Boucle de contrôle
        current_ticks = 0
        while current_ticks < target_ticks:
            # Ici on simule une lecture différentielle simplifiée
            current_ticks = abs(self.get_encoder_ticks() - start_ticks)
            time.sleep(0.05)

        # 4. Arrivée
        self.back_wheels.stop()
        self.front_wheels.turn(STEER_CENTER)
        
        # Mise à jour de la position interne (simplifiée)
        self.x = target_x
        self.y = target_y
        print("Cible atteinte.")

# --- EXEMPLE D'UTILISATION ---
if __name__ == "__main__":
    nav = PicarProNavigator()
    try:
        # Aller à 1 mètre devant et 0.5 mètre à droite
        nav.move_to(0.5, 1.0)
    except KeyboardInterrupt:
        nav.back_wheels.stop()