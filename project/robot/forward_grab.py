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

GRIPPER_OPEN = 80
GRIPPER_CLOSE = 0

def set_angle(ID, angle):
    angle = max(0, min(180, angle))
    s = servo.Servo(pca.channels[ID], min_pulse=500, max_pulse=2400, actuation_range=180)
    s.angle = angle

sequence = [
    # Step 0: Home — all at 90, gripper open
    (90, 90, 90, GRIPPER_OPEN),

    # Step 1: Reach down — shoulder to 180
    (90, 150, 100, GRIPPER_OPEN),

    # Step 2: Close gripper
    (90, 150, 90, GRIPPER_CLOSE),

    # Step 3: Back to home with object
    (90, 90, 90, GRIPPER_CLOSE),
    
    (90, 90, 90, GRIPPER_OPEN),

]

DELAY_BETWEEN_STEPS = 2.0

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
        set_angle(GRIPPER_CH, 90)
        set_angle(GRIPPER_CH, 80)

    finally:
        pca.deinit()
