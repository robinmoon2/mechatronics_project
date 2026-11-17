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


def set_angle(ID, angle):
    """ID 0 = Direction, ID 1 = Ultrason"""
    srv = servo.Servo(pca.channels[ID], min_pulse=500, max_pulse=2400)
    srv.angle = angle


def main():
    try:
        set_angle(4, 90)
        set_angle(2, 130)
        set_angle(1, 90)
        set_angle(3, 95)

        time.sleep(1)
        set_angle(4, 0)
        time.sleep(1)
        set_angle(2, 90)

    except KeyboardInterrupt:
        stopper()
        set_angle(0, 90)

if __name__ == "__main__":
    main()
