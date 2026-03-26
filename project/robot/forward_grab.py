# manual_grab.py
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x5f)
pca.frequency = 50

BASE_CH = 1
SHOULDER_CH = 2
ELBOW_CH = 3
GRIPPER_CH = 4

def set_angle(ID, angle):
    angle = max(0, min(180, angle))
    s = servo.Servo(pca.channels[ID], min_pulse=500, max_pulse=2400, actuation_range=180)
    s.angle = angle

# ============================================
# DEFINE YOUR SEQUENCE HERE
# Each step: (base, shoulder, elbow, gripper)
# ============================================
sequence = [
    # Step 0: Home position
    (90, 90, 90, 0),

    # Step 1: Open gripper, position above object
    (90, 134, 90, 0),
]

DELAY_BETWEEN_STEPS = 1.0  # seconds between each step

# ============================================
# EXECUTE
# ============================================
def run_sequence():
    for i, (base, shoulder, elbow, gripper) in enumerate(sequence):
        print(f"Step {i}: base={base} shoulder={shoulder} elbow={elbow} gripper={gripper}")
        set_angle(BASE_CH, base)
        set_angle(SHOULDER_CH, shoulder)
        set_angle(ELBOW_CH, elbow)
        set_angle(GRIPPER_CH, gripper)
        time.sleep(DELAY_BETWEEN_STEPS)
    print("DONE")

if __name__ == "__main__":
    try:
        run_sequence()
    except KeyboardInterrupt:
        print("Stopped")
        set_angle(BASE_CH, 90)
        set_angle(SHOULDER_CH, 90)
        set_angle(ELBOW_CH, 90)
        set_angle(GRIPPER_CH, 30)
    finally:
        pca.deinit()
