from controller import Robot
import random
import math
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

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

# ---------------- JOINT LIMITS ----------------
JOINT_LIMITS = [
    (-3.14, 3.14),
    (-3.14, 3.14),
    (-3.14, 3.14),
    (-3.14, 3.14),
    (-3.14, 3.14),
    (-3.14, 3.14),
]

# ---------------- OBSTACLE ZONE ----------------
OBSTACLE_ZONE = {
    "pan_min":   -0.5,
    "pan_max":    0.5,
    "lift_min":  -1.4,
    "lift_max":  -0.5,
    "elbow_min": -0.5,
    "elbow_max":  0.5,
}

def is_in_collision(config):
    pan   = config[0]
    lift  = config[1]
    elbow = config[2]
    if (OBSTACLE_ZONE["pan_min"]   < pan   < OBSTACLE_ZONE["pan_max"] and
        OBSTACLE_ZONE["lift_min"]  < lift  < OBSTACLE_ZONE["lift_max"] and
        OBSTACLE_ZONE["elbow_min"] < elbow < OBSTACLE_ZONE["elbow_max"]):
        return True
    return False

# ---------------- RRT ----------------
def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))

def random_config():
    return [random.uniform(lo, hi) for lo, hi in JOINT_LIMITS]

def steer(from_node, to_node, step=0.2):
    d = distance(from_node, to_node)
    if d < step:
        return to_node
    ratio = step / d
    return [f + ratio * (t - f) for f, t in zip(from_node, to_node)]

def rrt(start, goal, max_iter=2000):
    tree = [start]
    parent = {0: None}
    for i in range(max_iter):
        rand = goal if random.random() < 0.1 else random_config()
        nearest_idx = min(range(len(tree)), key=lambda i: distance(tree[i], rand))
        nearest = tree[nearest_idx]
        new_node = steer(nearest, rand)
        if is_in_collision(new_node):
            continue
        tree.append(new_node)
        parent[len(tree) - 1] = nearest_idx
        if distance(new_node, goal) < 0.3:
            path = [goal]
            idx = len(tree) - 1
            while idx is not None:
                path.append(tree[idx])
                idx = parent[idx]
            path.reverse()
            print(f"RRT found path in {i} iterations, {len(path)} waypoints")
            return path
    print("RRT failed, using direct path")
    return [start, goal]

# ---------------- CUBIC SPLINE SMOOTHING ----------------
def smooth_path(path, num_points=200):
    if len(path) < 3:
        print("Path too short to smooth, using raw path")
        return path
    path_array = np.array(path)
    t          = np.linspace(0, 1, len(path))
    t_smooth   = np.linspace(0, 1, num_points)
    smoothed   = []
    for joint_idx in range(6):
        cs = CubicSpline(t, path_array[:, joint_idx])
        smoothed.append(cs(t_smooth))
    smoothed_path = np.array(smoothed).T.tolist()
    safe_path = [p for p in smoothed_path if not is_in_collision(p)]
    print(f"Smoothed path: {len(smoothed_path)} points -> {len(safe_path)} safe points")
    return safe_path

# ---------------- MOVE ----------------
def move_to(position, steps=30):
    for i, m in enumerate(ur_motors):
        m.setPosition(position[i])
    for _ in range(steps):
        robot.step(TIME_STEP)

# ---------------- METRICS ----------------
def compute_metrics(path, label=""):
    total_length = 0
    max_jerk     = 0
    for i in range(1, len(path)):
        seg = distance(path[i], path[i-1])
        total_length += seg
        if i >= 2:
            jerk = distance(
                [path[i][j]   - path[i-1][j] for j in range(6)],
                [path[i-1][j] - path[i-2][j] for j in range(6)]
            )
            max_jerk = max(max_jerk, jerk)
    print(f"--- {label} METRICS ---")
    print(f"Total path length : {total_length:.4f} rad")
    print(f"Total waypoints   : {len(path)}")
    print(f"Max jerk          : {max_jerk:.6f}")
    print(f"----------------------")

# ---------------- VISUALIZE ----------------
def visualize(raw_path, smooth_path_data):
    raw    = np.array(raw_path)
    smooth = np.array(smooth_path_data)
    joint_names = [
        "shoulder_pan", "shoulder_lift", "elbow",
        "wrist_1", "wrist_2", "wrist_3"
    ]

    # 1. Joint Trajectories
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle("Joint Trajectories: Raw vs Smooth", fontsize=14)
    for i, ax in enumerate(axes.flat):
        t_raw    = np.linspace(0, 1, len(raw))
        t_smooth = np.linspace(0, 1, len(smooth))
        ax.plot(t_raw,    raw[:, i],    'r--', label='Raw RRT', linewidth=1.5)
        ax.plot(t_smooth, smooth[:, i], 'b-',  label='Smooth',  linewidth=2)
        ax.set_title(joint_names[i])
        ax.set_xlabel("Normalized Time")
        ax.set_ylabel("Joint Angle (rad)")
        ax.legend()
        ax.grid(True)
    plt.tight_layout()
    plt.savefig("trajectory.png", dpi=150)
    print("Saved: trajectory.png")
    plt.close()

    # 2. Velocity Profile
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle("Velocity Profile (Smooth Path)", fontsize=14)
    for i, ax in enumerate(axes.flat):
        vel = np.diff(smooth[:, i])
        ax.plot(vel, 'g-', linewidth=1.5)
        ax.set_title(f"{joint_names[i]} velocity")
        ax.set_xlabel("Waypoint")
        ax.set_ylabel("Delta angle / step")
        ax.grid(True)
    plt.tight_layout()
    plt.savefig("velocity.png", dpi=150)
    print("Saved: velocity.png")
    plt.close()

    # 3. Acceleration Profile
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle("Acceleration Profile (Smooth Path)", fontsize=14)
    for i, ax in enumerate(axes.flat):
        acc = np.diff(smooth[:, i], n=2)
        ax.plot(acc, 'm-', linewidth=1.5)
        ax.set_title(f"{joint_names[i]} acceleration")
        ax.set_xlabel("Waypoint")
        ax.set_ylabel("Delta^2 angle / step")
        ax.grid(True)
    plt.tight_layout()
    plt.savefig("acceleration.png", dpi=150)
    print("Saved: acceleration.png")
    plt.close()

    # 4. Path Length Comparison
    def path_length(p):
        return sum(distance(p[i], p[i-1]) for i in range(1, len(p)))

    fig, ax = plt.subplots(figsize=(6, 5))
    lengths = [path_length(raw_path), path_length(smooth_path_data)]
    bars    = ax.bar(["Raw RRT", "Smooth Spline"], lengths, color=["red", "blue"], width=0.4)
    ax.set_title("Path Length Comparison")
    ax.set_ylabel("Total Joint Distance (rad)")
    for bar, val in zip(bars, lengths):
        ax.text(bar.get_x() + bar.get_width()/2,
                bar.get_height() + 0.01,
                f"{val:.3f}", ha='center', fontsize=11)
    plt.tight_layout()
    plt.savefig("path_comparison.png", dpi=150)
    print("Saved: path_comparison.png")
    plt.close()

# ---------------- FINAL REPORT ----------------
def final_report(raw_path, smooth_path_data):
    def path_length(p):
        return sum(distance(p[i], p[i-1]) for i in range(1, len(p)))

    def max_jerk(p):
        jerk = 0
        for i in range(2, len(p)):
            j = distance(
                [p[i][k]   - p[i-1][k] for k in range(6)],
                [p[i-1][k] - p[i-2][k] for k in range(6)]
            )
            jerk = max(jerk, j)
        return jerk

    def avg_velocity(p):
        vels = [distance(p[i], p[i-1]) for i in range(1, len(p))]
        return sum(vels) / len(vels)

    def smoothness_score(p):
        jerks = []
        for i in range(2, len(p)):
            j = distance(
                [p[i][k]   - p[i-1][k] for k in range(6)],
                [p[i-1][k] - p[i-2][k] for k in range(6)]
            )
            jerks.append(j)
        return sum(jerks) / len(jerks) if jerks else 0

    raw_len       = path_length(raw_path)
    smooth_len    = path_length(smooth_path_data)
    raw_jerk      = max_jerk(raw_path)
    smooth_jerk   = max_jerk(smooth_path_data)
    raw_vel       = avg_velocity(raw_path)
    smooth_vel    = avg_velocity(smooth_path_data)
    raw_smooth    = smoothness_score(raw_path)
    smooth_smooth = smoothness_score(smooth_path_data)

    print("\n")
    print("=" * 50)
    print("        FINAL PERFORMANCE REPORT")
    print("=" * 50)
    print(f"{'Metric':<25} {'Raw RRT':>10} {'Smooth':>10}")
    print("-" * 50)
    print(f"{'Path Length (rad)':<25} {raw_len:>10.4f} {smooth_len:>10.4f}")
    print(f"{'Waypoints':<25} {len(raw_path):>10} {len(smooth_path_data):>10}")
    print(f"{'Max Jerk':<25} {raw_jerk:>10.6f} {smooth_jerk:>10.6f}")
    print(f"{'Avg Velocity':<25} {raw_vel:>10.6f} {smooth_vel:>10.6f}")
    print(f"{'Avg Smoothness Score':<25} {raw_smooth:>10.6f} {smooth_smooth:>10.6f}")
    print("=" * 50)

    improvement = ((raw_smooth - smooth_smooth) / raw_smooth * 100) if raw_smooth > 0 else 0
    print(f"  Smoothness improved by : {improvement:.1f}%")
    print(f"  Path length change     : {((smooth_len - raw_len) / raw_len * 100):+.1f}%")
    print("=" * 50)
    print("\n")

    # Summary Chart
    metrics       = ["Path Length", "Max Jerk", "Avg Velocity", "Avg Smoothness"]
    raw_values    = [raw_len, raw_jerk, raw_vel, raw_smooth]
    smooth_values = [smooth_len, smooth_jerk, smooth_vel, smooth_smooth]

    x   = range(len(metrics))
    fig, ax = plt.subplots(figsize=(10, 6))
    bars1 = ax.bar([i - 0.2 for i in x], raw_values,    width=0.4, label="Raw RRT",       color="red",  alpha=0.8)
    bars2 = ax.bar([i + 0.2 for i in x], smooth_values, width=0.4, label="Smooth Spline", color="blue", alpha=0.8)
    ax.set_title("Final Metrics: Raw RRT vs Smooth Spline", fontsize=14)
    ax.set_xticks(list(x))
    ax.set_xticklabels(metrics)
    ax.set_ylabel("Value")
    ax.legend()
    ax.grid(axis='y', linestyle='--', alpha=0.5)
    for bar in bars1:
        ax.text(bar.get_x() + bar.get_width()/2,
                bar.get_height() + 0.001,
                f"{bar.get_height():.3f}", ha='center', fontsize=8)
    for bar in bars2:
        ax.text(bar.get_x() + bar.get_width()/2,
                bar.get_height() + 0.001,
                f"{bar.get_height():.3f}", ha='center', fontsize=8)
    plt.tight_layout()
    plt.savefig("final_report.png", dpi=150)
    print("Saved: final_report.png")
    plt.close()

# ================================================================
#                           MAIN
# ================================================================
START = [1.5,   0.0,  0.0, 0.0, 0.0, 0.0]
GOAL  = [-1.5, -0.05, 0.0, 0.0, 0.0, 0.0]

for _ in range(30):
    robot.step(TIME_STEP)

print("Closing gripper (PICK)")
gripper_close()

print("Running RRT...")
raw_path = rrt(START, GOAL)

print("Smoothing path with Cubic Spline...")
smooth = smooth_path(raw_path, num_points=200)

compute_metrics(raw_path, "RAW PATH")
compute_metrics(smooth,   "SMOOTH PATH")

print(f"Executing smooth path ({len(smooth)} waypoints)...")
for idx, waypoint in enumerate(smooth):
    move_to(waypoint, steps=30)

print("Opening gripper (PLACE)")
gripper_open()
print("Done")

print("Generating plots...")
visualize(raw_path, smooth)

print("Generating final report...")
final_report(raw_path, smooth)
print("All files saved in controller folder.")