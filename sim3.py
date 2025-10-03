import pybullet as p
import pybullet_data
import time
import numpy as np

# Initialize PyBullet in GUI mode
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# Camera setup (top-down)
p.resetDebugVisualizerCamera(cameraDistance=8,
                             cameraYaw=0,
                             cameraPitch=-89.999,
                             cameraTargetPosition=[0, 0, 0])

# Parameters
num_bots = 3
bot_radius = 0.1
bot_height = 0.2
min_dist = 0.7  # Minimum allowed distance between bots (70 cm)
speed = 0.01
start_positions = [(-2, -5), (0, -5), (2, -5)]  # red, green, blue start
target_positions = [(2, 1), (0, 1), (-2, 1)]    # red, green, blue target
bot_colors = [
    [1, 0, 0, 1],  # Red
    [0, 1, 0, 1],  # Green
    [0, 0, 1, 1],  # Blue
]

# Priority: blue (highest=0), red (medium=1), green (lowest=2)
# Since bot order is red(0), green(1), blue(2), assign accordingly:
priority = [1, 2, 0]  # indices correspond to bot indices

bot_ids = []

# Create bots with color
for i, pos in enumerate(start_positions):
    colShape = p.createCollisionShape(p.GEOM_CYLINDER, radius=bot_radius, height=bot_height)
    visShape = p.createVisualShape(p.GEOM_CYLINDER, radius=bot_radius, length=bot_height, rgbaColor=bot_colors[i])
    bot_id = p.createMultiBody(baseMass=1,
                               baseCollisionShapeIndex=colShape,
                               baseVisualShapeIndex=visShape,
                               basePosition=[pos[0], pos[1], bot_height / 2])
    bot_ids.append(bot_id)

def get_bot_positions(bot_ids):
    """Returns Nx2 numpy array of bot XY positions."""
    return np.array([p.getBasePositionAndOrientation(bid)[0][:2] for bid in bot_ids])

def move_bots_with_priority(bot_ids, targets, speed, min_dist, priority):
    positions = get_bot_positions(bot_ids)
    for i, bot_id in enumerate(bot_ids):
        current = positions[i]
        target = np.array(targets[i])
        direction = target - current
        dist_to_target = np.linalg.norm(direction)
        if dist_to_target < 0.01:
            continue
        direction = direction / dist_to_target

        repulsion = np.zeros(2)
        must_yield = False

        for j, other_pos in enumerate(positions):
            if i != j:
                diff = current - other_pos
                dist = np.linalg.norm(diff)
                if dist < min_dist and dist > 0:
                    # Yield if other bot has higher priority
                    if priority[j] < priority[i]:
                        must_yield = True
                    repulsion += (diff / dist) * (1 - dist / min_dist)

        if must_yield:
            # Yield by reducing speed drastically or stopping
            move_vec = repulsion
            if np.linalg.norm(move_vec) > 0:
                move_vec = move_vec / np.linalg.norm(move_vec) * (speed * 0.1)
            else:
                move_vec = np.zeros(2)
        else:
            # Normal movement with repulsion to keep distance
            move_vec = direction + repulsion * 0.5
            if np.linalg.norm(move_vec) > 0:
                move_vec = move_vec / np.linalg.norm(move_vec) * speed
            else:
                move_vec = np.zeros(2)

        new_pos = current + move_vec
        p.resetBasePositionAndOrientation(bot_id, [new_pos[0], new_pos[1], bot_height / 2], [0, 0, 0, 1])

# Simulation loop
for _ in range(1000):
    move_bots_with_priority(bot_ids, target_positions, speed, min_dist, priority)
    p.stepSimulation()
    time.sleep(1. / 240.)

