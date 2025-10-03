import pybullet as p
import pybullet_data
import time
import numpy as np

# Start PyBullet in DIRECT mode (non-graphical)
physicsClient = p.connect(p.GUI)

# Load plane and set gravity
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")
p.resetDebugVisualizerCamera(cameraDistance=3,
                             cameraYaw=0,
                             cameraPitch=-89.999,
                             cameraTargetPosition=[0, 0, 0])


# Parameters
num_bots = 3
bot_radius = 0.1
bot_height = 0.2
start_positions = [(-1, -1), (0, -1), (1, -1)]
target_positions = [(1, 1), (0, 1), (-1, 1)]
bot_ids = []

# Create simple cylindrical bots
for pos in start_positions:
    colSphereId = p.createCollisionShape(p.GEOM_CYLINDER, radius=bot_radius, height=bot_height)
    visualShapeId = p.createVisualShape(p.GEOM_CYLINDER, radius=bot_radius, length=bot_height, rgbaColor=[0, 0, 1, 1])
    bot_id = p.createMultiBody(baseMass=1,
                               baseCollisionShapeIndex=colSphereId,
                               baseVisualShapeIndex=visualShapeId,
                               basePosition=[pos[0], pos[1], bot_height/2])
    bot_ids.append(bot_id)

# Function to step bots toward targets
def move_bots(bot_ids, target_positions, speed=0.01):
    for i, bot_id in enumerate(bot_ids):
        current_pos = np.array(p.getBasePositionAndOrientation(bot_id)[0][:2])
        target = np.array(target_positions[i])
        direction = target - current_pos
        distance = np.linalg.norm(direction)
        if distance > 0.01:
            direction = direction / distance
            new_pos = current_pos + direction * speed
            p.resetBasePositionAndOrientation(bot_id, [new_pos[0], new_pos[1], bot_height/2], [0, 0, 0, 1])

# Simulate movement
for step in range(500):
    move_bots(bot_ids, target_positions)
    p.stepSimulation()
    time.sleep(1. / 240.)

# Get final positions
final_positions = [p.getBasePositionAndOrientation(bot_id)[0] for bot_id in bot_ids]

# Disconnect
p.disconnect()
final_positions

