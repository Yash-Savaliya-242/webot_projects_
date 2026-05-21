from controller import Robot, Motor, DistanceSensor, PositionSensor
TIME_STEP = 32
# States
WAITING, GRASPING, ROTATING, RELEASING, ROTATING_BACK = range(5)
robot = Robot()
counter = 0
state = WAITING
speed = 1.0
detected_color = None
# Target positions - TUNE THESE
target_positions_yellow = [-1.88, -2.14, -2.38, -1.51]  # above plastic crate(1)
target_positions_red    = [-1.88, 2.14, -2.38, -1.51]  # above plastic crate
# --- Gripper motors ---
hand_motors = [
    robot.getDevice("finger_1_joint_1"),
    robot.getDevice("finger_2_joint_1"),
    robot.getDevice("finger_middle_joint_1"),
]
# --- UR5e arm motors ---
ur_motors = [
    robot.getDevice("shoulder_lift_joint"),
    robot.getDevice("elbow_joint"),
    robot.getDevice("wrist_1_joint"),
    robot.getDevice("wrist_2_joint"),
]
# Set speed
for m in ur_motors:
    m.setVelocity(speed)
# --- Sensors ---
distance_sensor = robot.getDevice("distance sensor")
distance_sensor.enable(TIME_STEP)
s1 = robot.getDevice("shoulder_lift_joint_sensor")
s1.enable(TIME_STEP)
s2 = robot.getDevice("elbow_joint_sensor")
s2.enable(TIME_STEP)
s3 = robot.getDevice("wrist_1_joint_sensor")
s3.enable(TIME_STEP)
s4 = robot.getDevice("wrist_2_joint_sensor")
s4.enable(TIME_STEP)
# --- Camera + Recognition ---
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
camera.recognitionEnable(TIME_STEP)
# --- Main loop ---
while robot.step(TIME_STEP) != -1:
    # --- Recognition ONLY in WAITING state so color never gets overwritten ---
    if state == WAITING:
        objects = camera.getRecognitionObjects()
        for obj in objects:
            colors = obj.getColors()
            r, g, b = colors[0], colors[1], colors[2]
            if r > 0.5 and g > 0.5 and b < 0.3:
                detected_color = "Yellow"
            elif r > 0.5 and g < 0.3 and b < 0.3:
                detected_color = "Red"
            elif r < 0.3 and g < 0.3 and b > 0.5:
                detected_color = "Blue"
            elif r < 0.3 and g > 0.5 and b < 0.3:
                detected_color = "Green"
            elif r > 0.5 and g < 0.3 and b > 0.5:
                detected_color = "Purple"
            elif r > 0.5 and g > 0.3 and b < 0.3:
                detected_color = "Orange"
            else:
                detected_color = f"Unknown RGB({r:.1f},{g:.1f},{b:.1f})"

    # --- GRASPING state — wait until color is detected ---
    if state == GRASPING and detected_color is None:
        objects = camera.getRecognitionObjects()
        for obj in objects:
            colors = obj.getColors()
            r, g, b = colors[0], colors[1], colors[2]
            if r > 0.5 and g > 0.5 and b < 0.3:
                detected_color = "Yellow"
            elif r > 0.5 and g < 0.3 and b < 0.3:
                detected_color = "Red"
            elif r < 0.3 and g < 0.3 and b > 0.5:
                detected_color = "Blue"
            elif r < 0.3 and g > 0.5 and b < 0.3:
                detected_color = "Green"
            elif r > 0.5 and g < 0.3 and b > 0.5:
                detected_color = "Purple"
            elif r > 0.5 and g > 0.3 and b < 0.3:
                detected_color = "Orange"

    if counter <= 0:
        if state == WAITING:
            if distance_sensor.getValue() < 500:
                detected_color = None  # reset for new can
                state = GRASPING
                counter = 8
                print("Grasping object")
                for m in hand_motors:
                    m.setPosition(0.85)
        elif state == GRASPING:
            if detected_color is None:
                pass  # keep waiting for color detection
            else:
                print(f"Detected: {detected_color}")
                if detected_color == "Yellow":
                    for i in range(4):
                        ur_motors[i].setPosition(target_positions_yellow[i])
                    print("Rotating to Yellow obj holder")
                else:
                    for i in range(4):
                        ur_motors[i].setPosition(target_positions_red[i])
                    print("Rotating to Red obj holder")
                counter = 80
                state = ROTATING
        elif state == ROTATING:
            counter = 8
            print("Releasing object")
            state = RELEASING
            for m in hand_motors:
                m.setPosition(0.05)
        elif state == RELEASING:
            for m in ur_motors:
                m.setPosition(0.0)
            print("Rotating arm back")
            counter = 80
            state = ROTATING_BACK
        elif state == ROTATING_BACK:
            state = WAITING
            print("Waiting")
    counter -= 1