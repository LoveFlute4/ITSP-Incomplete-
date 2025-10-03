import pybullet as p
import pybullet_data
import time
import numpy as np
import math

# Initialize PyBullet in GUI mode
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# Camera setup (top-down)
p.resetDebugVisualizerCamera(cameraDistance=12,
                             cameraYaw=0,
                             cameraPitch=-89.999,
                             cameraTargetPosition=[0, 0, 0])

# Parameters
num_bots = 15
bot_radius = 0.1
bot_height = 0.2
min_dist = 0.7  # Minimum allowed distance between bots (70 cm)
speed = 0.02
target_radius = 1.0  # Target region radius
bot_color = [0, 0, 1, 1]  # Blue color for all bots

# Initial target position (center of coordinate system)
target_center = np.array([0.0, 0.0])

# Create initial bot positions in a grid formation
def create_grid_positions(num_bots, spacing=1.5):
    """Create initial positions in a rough grid formation."""
    positions = []
    cols = int(math.ceil(math.sqrt(num_bots)))
    rows = int(math.ceil(num_bots / cols))
    
    start_x = -(cols - 1) * spacing / 2
    start_y = -6  # Start them away from center
    
    for i in range(num_bots):
        row = i // cols
        col = i % cols
        x = start_x + col * spacing
        y = start_y - row * spacing
        positions.append((x, y))
    
    return positions

start_positions = create_grid_positions(num_bots)
bot_ids = []

# Create bots with same color
for i, pos in enumerate(start_positions):
    colShape = p.createCollisionShape(p.GEOM_CYLINDER, radius=bot_radius, height=bot_height)
    visShape = p.createVisualShape(p.GEOM_CYLINDER, radius=bot_radius, length=bot_height, rgbaColor=bot_color)
    bot_id = p.createMultiBody(baseMass=1,
                               baseCollisionShapeIndex=colShape,
                               baseVisualShapeIndex=visShape,
                               basePosition=[pos[0], pos[1], bot_height / 2])
    bot_ids.append(bot_id)

# Create target region visualization (translucent sphere)
target_vis_shape = p.createVisualShape(p.GEOM_SPHERE, radius=target_radius, rgbaColor=[1, 1, 0, 0.3])
target_marker_id = p.createMultiBody(baseMass=0,
                                   baseVisualShapeIndex=target_vis_shape,
                                   basePosition=[target_center[0], target_center[1], 0.1])

def get_bot_positions(bot_ids):
    """Returns Nx2 numpy array of bot XY positions."""
    return np.array([p.getBasePositionAndOrientation(bid)[0][:2] for bid in bot_ids])

def get_mouse_click():
    """Check for mouse clicks and return world coordinates."""
    mouse_events = p.getMouseEvents()
    for event in mouse_events:
        if event[0] == 2 and event[3] == 0:  # Left mouse button pressed
            # Get camera info for coordinate conversion
            cam_info = p.getDebugVisualizerCamera()
            width, height = cam_info[0], cam_info[1]
            
            # Convert mouse coordinates to world coordinates
            mouse_x, mouse_y = event[1], event[2]
            
            # Simple conversion assuming top-down view
            # This is an approximation - for exact conversion, use projection matrices
            world_x = (mouse_x / width - 0.5) * cam_info[10] * 2
            world_y = (mouse_y / height - 0.5) * cam_info[10] * 2
            
            return np.array([world_x, -world_y])  # Flip Y for correct orientation
    return None

def calculate_dynamic_priority(positions, target_center):
    """Calculate priority based on distance to target center (closest = highest priority)."""
    distances = [np.linalg.norm(pos - target_center) for pos in positions]
    # Create priority list where smaller indices = higher priority
    # Sort by distance and get the indices
    sorted_indices = np.argsort(distances)
    priority = np.zeros(len(positions), dtype=int)
    for rank, bot_idx in enumerate(sorted_indices):
        priority[bot_idx] = rank
    return priority

def is_in_target_region(position, target_center, target_radius):
    """Check if a position is within the target region."""
    return np.linalg.norm(position - target_center) <= target_radius

def move_bots_with_dynamic_priority(bot_ids, target_center, target_radius, speed, min_dist):
    positions = get_bot_positions(bot_ids)
    priority = calculate_dynamic_priority(positions, target_center)
    
    for i, bot_id in enumerate(bot_ids):
        current = positions[i]
        
        # Check if bot is already in target region
        if is_in_target_region(current, target_center, target_radius):
            # Bot reached target - minimal movement or stop
            continue
        
        # Calculate direction to target center
        direction = target_center - current
        dist_to_target = np.linalg.norm(direction)
        if dist_to_target > 0:
            direction = direction / dist_to_target
        else:
            continue

        repulsion = np.zeros(2)
        must_yield = False

        # Check interactions with other bots
        for j, other_pos in enumerate(positions):
            if i != j:
                diff = current - other_pos
                dist = np.linalg.norm(diff)
                if dist < min_dist and dist > 0:
                    # Yield if other bot has higher priority (lower priority value)
                    if priority[j] < priority[i]:
                        must_yield = True
                    # Add repulsion force
                    repulsion += (diff / dist) * (1 - dist / min_dist) * 2.0

        if must_yield:
            # Yield by moving away or stopping
            move_vec = repulsion
            if np.linalg.norm(move_vec) > 0:
                move_vec = move_vec / np.linalg.norm(move_vec) * (speed * 0.2)
            else:
                move_vec = np.zeros(2)
        else:
            # Normal movement toward target with collision avoidance
            move_vec = direction + repulsion * 0.3
            if np.linalg.norm(move_vec) > 0:
                move_vec = move_vec / np.linalg.norm(move_vec) * speed
            else:
                move_vec = np.zeros(2)

        new_pos = current + move_vec
        p.resetBasePositionAndOrientation(bot_id, [new_pos[0], new_pos[1], bot_height / 2], [0, 0, 0, 1])

# Simulation loop
print("Click in the PyBullet window to set new target locations!")
print("Bots will move toward the clicked location with dynamic priority based on distance.")

for step in range(10000):  # Extended simulation
    # Check for mouse clicks to update target
    new_target = get_mouse_click()
    if new_target is not None:
        target_center = new_target
        print(f"New target set at: ({target_center[0]:.2f}, {target_center[1]:.2f})")
        # Update target marker position
        p.resetBasePositionAndOrientation(target_marker_id, 
                                        [target_center[0], target_center[1], 0.1], 
                                        [0, 0, 0, 1])
    
    # Move bots with dynamic priority
    move_bots_with_dynamic_priority(bot_ids, target_center, target_radius, speed, min_dist)
    
    # Step simulation
    p.stepSimulation()
    time.sleep(1. / 240.)
    
    # Print status every 100 steps
    if step % 100 == 0:
        positions = get_bot_positions(bot_ids)
        bots_in_target = sum(1 for pos in positions if is_in_target_region(pos, target_center, target_radius))
        print(f"Step {step}: {bots_in_target}/{num_bots} bots in target region")

p.disconnect()
