# arm_ik.py
import math
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

# Robot dimensions (cm) - matching PUMA notation
d1 = 8.5    # base to shoulder height (vertical offset)
a2 = 11.0     # upper arm length (shoulder to elbow)
a3 = 13.0    # forearm length (elbow to end effector)

def set_angle(ID, angle):
    angle = max(0, min(180, angle))
    s = servo.Servo(pca.channels[ID], min_pulse=500, max_pulse=2400, actuation_range=180)
    s.angle = angle

def inverse_kinematics(px, py, pz):
    """
    PUMA-style 3DOF IK.
    px, py, pz: target position in cm from base frame on ground.
    Returns (servo1, servo2, servo3) in degrees or None.
    """
    # === THETA 1: Base rotation ===
    theta1 = math.atan2(py, px)
    
    # === Project into vertical plane ===
    r = math.sqrt(px**2 + py**2)   # horizontal distance
    # === THETA 3: Elbow (2R planar IK) ===
    D = (r**2 + pz**2 - a2**2 - a3**2) / (2 * a2 * a3)

    # Check reachability
    if abs(D) > 1.0:
        dist = math.sqrt(r**2 + pz**2)
        print(f"  UNREACHABLE: D={D:.3f}, dist={dist:.1f}, range=[{abs(a2-a3):.1f}, {a2+a3:.1f}]")
        return None

    # Elbow down solution (use -sqrt for elbow up)
    theta3 = math.atan2(math.sqrt(1 - D**2), D)

    # === THETA 2: Shoulder ===
    theta2 = math.atan2(pz, r) - math.atan2(a3 * math.sin(theta3), a2 + a3 * math.cos(theta3))

    # Convert to degrees
    t1_deg = math.degrees(theta1)
    t2_deg = math.degrees(theta2)
    t3_deg = math.degrees(theta3)

    print(f"  IK angles: theta1={t1_deg:.1f}° theta2={t2_deg:.1f}° theta3={t3_deg:.1f}°")

    servo1 = 90 + t1_deg          # base: 90=forward, positive theta1=left
    servo2 = 90 - t2_deg          # shoulder: 90=horizontal, negative theta2=reaching down
    servo3 = 90 + t3_deg          # elbow: 90=straight, negative theta3=bent down

    print(f"  Servo angles: base={servo1:.1f} shoulder={servo2:.1f} elbow={servo3:.1f}")

    # Clamp check
    for name, val in [("base", servo1), ("shoulder", servo2), ("elbow", servo3)]:
        if val < 0 or val > 180:
            print(f"  WARNING: {name} servo = {val:.1f}° out of range!")

    return servo1, servo2, servo3

def move_to(px, py, pz, grip_open=True):
    result = inverse_kinematics(px, py, pz)
    if result is None:
        return False
    s1, s2, s3 = result
    set_angle(BASE_CH, s1)
    time.sleep(0.3)
    set_angle(SHOULDER_CH, s2)
    time.sleep(0.3)
    set_angle(ELBOW_CH, s3)
    time.sleep(0.3)
    set_angle(GRIPPER_CH, 30 if grip_open else 120)
    time.sleep(0.3)
    return True

def home():
    print("--- Homing (all 90) ---")
    set_angle(BASE_CH, 90)
    set_angle(SHOULDER_CH, 90)
    set_angle(ELBOW_CH, 90)
    set_angle(GRIPPER_CH, 90)
    time.sleep(0.5)

def grab_at(px, py, pz):
    print("--- Opening gripper ---")
    set_angle(GRIPPER_CH, 30)
    time.sleep(0.5)
    print("--- Moving to target ---")
    if not move_to(px, py, pz, grip_open=True):
        return False
    time.sleep(0.5)
    print("--- Closing gripper ---")
    set_angle(GRIPPER_CH, 120)
    time.sleep(0.5)
    print("--- Lifting ---")
    move_to(px, py, pz + 10, grip_open=False)
    return True

# ============ TEST ============
if __name__ == "__main__":
    print("=== PUMA-STYLE 3DOF IK ===")
    print(f"d1={d1}cm, a2={a2}cm, a3={a3}cm")
    print(f"Reach: [{abs(a2-a3):.1f}, {a2+a3:.1f}]cm from shoulder\n")

    # First calibrate servo directions
    print("STEP 1: Calibration")
    home()
    input("All servos at 90. Arm should be horizontal+straight. Press Enter...")

    print("\nSetting shoulder to 60...")
    set_angle(SHOULDER_CH, 60)
    resp = input("Did arm go UP or DOWN? (u/d): ").strip().lower()
    home()
    time.sleep(0.3)

    print("Setting elbow to 60...")
    set_angle(ELBOW_CH, 60)
    resp2 = input("Did forearm go UP or DOWN? (u/d): ").strip().lower()
    home()
    time.sleep(0.3)

    print(f"\nShoulder 60 => {resp}, Elbow 60 => {resp2}")
    print("If signs are wrong, flip the +/- in the servo mapping section.\n")

    # Test positions
    tests = [
        (20, 0, 0,   "ground, 20cm forward"),
        (15, 0, 0,   "ground, 15cm forward"),
        (20, 0, 10,  "10cm high, 20cm forward"),
        (15, 0, 19,  "shoulder height, 15cm forward"),
        (10, 10, 0,  "ground, diagonal left"),
    ]

    try:
        for px, py, pz, desc in tests:
            print(f"\n--- ({px},{py},{pz}): {desc} ---")
            move_to(px, py, pz)
            input("Press enter for next...")
        #home()
    except KeyboardInterrupt:
        home()
    finally:
        pca.deinit()
