from controller import Robot
TIME_STEP = 32
robot = Robot()

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
    m.setVelocity(1.0)

# ---------------- POSITIONS ----------------
positions = [
    [1.5,  0.0,   0.0, 0.0, 0.0, 0.0],   # step 1
    [1.5, -1.57,  0.0, 0.0, 0.0, 0.0],   # step 2
    [-1.5, -1.57, 0.0, 0.0, 0.0, 0.0],   # step 3
    [-1.5, -0.001, 0.0, 0.0, 0.0, 0.0],   # step 4 - above hole
]

up_pos   = [-1.5, -0.5,  0.0, 0.0, 0.0, 0.0]  # same pan/elbow as step4, just shoulder up
down_pos = [-1.5, -0.001, 0.0, 0.0, 0.0, 0.0] # exact same as step4

# ---------------- MAIN ----------------
current = 0

for _ in range(30):
    robot.step(TIME_STEP)

while robot.step(TIME_STEP) != -1:
    if current < len(positions):
        pos = positions[current]
        print(f"Moving step {current}")
        for i, m in enumerate(ur_motors):
            m.setPosition(pos[i])
        for _ in range(150):
            robot.step(TIME_STEP)
        current += 1
    else:
        hit_total = 5
        hit_count = 0
        while hit_count < hit_total:
            hit_count += 1
            print(f"Hit {hit_count}/{hit_total} — DOWN")
            for i, m in enumerate(ur_motors):
                m.setPosition(down_pos[i])
            for _ in range(60):
                robot.step(TIME_STEP)
            print(f"Hit {hit_count}/{hit_total} — UP")
            for i, m in enumerate(ur_motors):
                m.setPosition(up_pos[i])
            for _ in range(60):
                robot.step(TIME_STEP)
        print("Done — 5 hits completed")
        break