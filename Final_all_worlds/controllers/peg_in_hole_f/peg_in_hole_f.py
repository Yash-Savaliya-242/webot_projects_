from controller import Robot
TIME_STEP = 32
robot = Robot()
# ---------------- CAMERA ----------------
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
# ---------------- ARM ----------------
ur_motors = [
    robot.getDevice("shoulder_pan_joint"),
    robot.getDevice("shoulder_lift_joint"),
    robot.getDevice("elbow_joint"),
    robot.getDevice("wrist_1_joint"),
    robot.getDevice("wrist_2_joint"),
    robot.getDevice("wrist_3_joint"),
]
for m in ur_motors:
    m.setVelocity(0.5)
# ---------------- GRIPPER ----------------
hand_motors = [
    robot.getDevice("finger_1_joint_1"),
    robot.getDevice("finger_2_joint_1"),
    robot.getDevice("finger_middle_joint_1"),
]
for m in hand_motors:
    m.setVelocity(0.5)
def gripper_close():
    for m in hand_motors:
        m.setPosition(0.7)
    for _ in range(80):
        robot.step(TIME_STEP)
def gripper_open():
    for m in hand_motors:
        m.setPosition(0.05)
    for _ in range(80):
        robot.step(TIME_STEP)
# ---------------- POSITIONS ----------------
positions = [
    [1.5,  0.0,  0.0, 0.0, 0.0, 0.0],
    [1.5, -1.57, 0.0, 0.0, 0.0, 0.0],
    [-1.5, -1.57, 0.0, 0.0, 0.0, 0.0],
    [-1.5, -0.2,  0.0, 0.0, 0.0, 0.0],
]
# ---------------- MAIN ----------------
current = 0
for _ in range(30):
    robot.step(TIME_STEP)
print("Closing gripper (PICK)")
gripper_close()
while robot.step(TIME_STEP) != -1:
    image = camera.getImage()  # camera captures every step
    if current < len(positions):
        pos = positions[current]
        print(f"Moving step {current}")
        for i, m in enumerate(ur_motors):
            m.setPosition(pos[i])
        for _ in range(150):
            robot.step(TIME_STEP)
        current += 1
    else:
        print("Opening gripper (DROP)")
        gripper_open()
        print("Done")
        break