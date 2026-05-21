from controller import Robot, DistanceSensor, Motor, PositionSensor

TIME_STEP = 32
robot = Robot()

hand_motors = [
    robot.getDevice("finger_1_joint_1"),
    robot.getDevice("finger_2_joint_1"),
    robot.getDevice("finger_middle_joint_1"),
]

ur_motors = [
    robot.getDevice("shoulder_lift_joint"),
    robot.getDevice("elbow_joint"),
    robot.getDevice("wrist_1_joint"),
    robot.getDevice("wrist_2_joint"),
]

for m in ur_motors:
    m.setVelocity(1.0)

distance_sensor = robot.getDevice("distance sensor")
distance_sensor.enable(TIME_STEP)

position_sensor = robot.getDevice("wrist_1_joint_sensor")
position_sensor.enable(TIME_STEP)

target_positions = [-1.88, -2.14, -2.38, -1.51]

WAITING, GRASPING, ROTATING, RELEASING, ROTATING_BACK = 0, 1, 2, 3, 4
state = WAITING
counter = 0

while robot.step(TIME_STEP) != -1:
    if counter <= 0:
        if state == WAITING:
            if distance_sensor.getValue() < 500:
                state = GRASPING
                counter = 8
                print("Grasp")
                for m in hand_motors:
                    m.setPosition(0.85)

        elif state == GRASPING:
            for i, m in enumerate(ur_motors):
                m.setPosition(target_positions[i])
            state = ROTATING

        elif state == ROTATING:
            if position_sensor.getValue() < -2.3:
                counter = 8
                print("Release")
                state = RELEASING
                for m in hand_motors:
                    m.setPosition(m.getMinPosition())

        elif state == RELEASING:
            for m in ur_motors:
                m.setPosition(0.0)
            state = ROTATING_BACK

        elif state == ROTATING_BACK:
            if position_sensor.getValue() > -0.1:
                state = WAITING

    counter -= 1