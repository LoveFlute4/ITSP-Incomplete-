import pybullet as p
import pybullet_data
import time
import numpy as np
import math
from collections import deque

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
min_dist = 0.4  # Minimum allowed distance between bots (40 cm)
speed = 0.03
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
            world_x = (mouse_x / width - 0.5) * cam_info[10] * 2
            world_y = (mouse_y / height - 0.5) * cam_info[10] * 2
            
            return np.array([world_x, -world_y])  # Flip Y for correct orientation
    return None

def calculate_dynamic_priority(positions, target_center):
    """Calculate priority based on distance to target center (closest = highest priority)."""
    distances = [np.linalg.norm(pos - target_center) for pos in positions]
    sorted_indices = np.argsort(distances)
    priority = np.zeros(len(positions), dtype=int)
    for rank, bot_idx in enumerate(sorted_indices):
        priority[bot_idx] = rank
    return priority

def is_in_target_region(position, target_center, target_radius):
    """Check if a position is within the target region."""
    return np.linalg.norm(position - target_center) <= target_radius

def find_alternative_path(current_pos, target_center, blocking_positions, min_dist):
    """Find alternative path around blocking bots using simple pathfinding."""
    # Generate potential waypoints around blocking bots
    waypoints = []
    
    for block_pos in blocking_positions:
        # Create waypoints around each blocking bot
        angles = np.linspace(0, 2*np.pi, 8, endpoint=False)
        for angle in angles:
            offset = np.array([np.cos(angle), np.sin(angle)]) * (min_dist * 1.2)
            waypoint = block_pos + offset
            waypoints.append(waypoint)
    
    if not waypoints:
        return target_center  # No blocking bots, go directly to target
    
    # Find the best waypoint (closest to straight line to target)
    direct_to_target = target_center - current_pos
    best_waypoint = None
    best_score = float('inf')
    
    for waypoint in waypoints:
        # Calculate path: current -> waypoint -> target
        to_waypoint = waypoint - current_pos
        waypoint_to_target = target_center - waypoint
        
        # Score based on total distance and deviation from direct path
        total_dist = np.linalg.norm(to_waypoint) + np.linalg.norm(waypoint_to_target)
        
        # Check if waypoint is clear of other bots
        waypoint_clear = True
        for block_pos in blocking_positions:
            if np.linalg.norm(waypoint - block_pos) < min_dist:
                waypoint_clear = False
                break
        
        if waypoint_clear and total_dist < best_score:
            best_score = total_dist
            best_waypoint = waypoint
    
    return best_waypoint if best_waypoint is not None else target_center

def detect_blocking_bots(current_pos, target_pos, all_positions, current_idx, min_dist, priority):
    """Detect bots that are blocking the direct path to target."""
    direct_vec = target_pos - current_pos
    direct_dist = np.linalg.norm(direct_vec)
    
    if direct_dist == 0:
        return []
    
    direct_unit = direct_vec / direct_dist
    blocking_bots = []
    
    for j, other_pos in enumerate(all_positions):
        if j == current_idx:
            continue
            
        # Vector from current bot to other bot
        to_other = other_pos - current_pos
        
        # Project onto direct path vector
        projection_length = np.dot(to_other, direct_unit)
        
        # Check if other bot is in front and within blocking distance
        if 0 < projection_length < direct_dist:
            # Distance from other bot to direct path line
            projection_point = current_pos + direct_unit * projection_length
            perpendicular_dist = np.linalg.norm(other_pos - projection_point)
            
            # If bot is close to the direct path and has higher priority
            if perpendicular_dist < min_dist * 1.5 and priority[j] <= priority[current_idx]:
                blocking_bots.append(other_pos)
    
    return blocking_bots

def optimize_position_in_target(current_pos, target_center, target_radius, all_positions, current_idx, min_dist):
    """Optimize position within target region for better spacing."""
    # Find a position within target that maximizes distance to other bots
    best_pos = current_pos
    best_min_dist = 0
    
    # Try several positions around current location within target region
    angles = np.linspace(0, 2*np.pi, 12, endpoint=False)
    for angle in angles:
        for radius in [0.1, 0.2, 0.3]:  # Small adjustments
            test_pos = current_pos + np.array([np.cos(angle), np.sin(angle)]) * radius
            
            # Check if still within target region
            if np.linalg.norm(test_pos - target_center) <= target_radius:
                # Calculate minimum distance to other bots
                min_dist_to_others = float('inf')
                for j, other_pos in enumerate(all_positions):
                    if j != current_idx:
                        dist = np.linalg.norm(test_pos - other_pos)
                        min_dist_to_others = min(min_dist_to_others, dist)
                
                if min_dist_to_others > best_min_dist:
                    best_min_dist = min_dist_to_others
                    best_pos = test_pos
    
    return best_pos

def move_bots_with_pathfinding(bot_ids, target_center, target_radius, speed, min_dist):
    positions = get_bot_positions(bot_ids)
    priority = calculate_dynamic_priority(positions, target_center)
    
    for i, bot_id in enumerate(bot_ids):
        current = positions[i]
        in_target = is_in_target_region(current, target_center, target_radius)
        
        if in_target:
            # Bot is in target region - optimize position for spacing
            optimized_pos = optimize_position_in_target(current, target_center, target_radius, 
                                                      positions, i, min_dist)
            if np.linalg.norm(optimized_pos - current) > 0.01:
                direction = optimized_pos - current
                direction = direction / np.linalg.norm(direction)
                move_vec = direction * speed * 0.5  # Slower movement within target
            else:
                continue  # Stay in place if well positioned
        else:
            # Bot is outside target region - navigate towards it
            # Check for blocking bots
            blocking_bots = detect_blocking_bots(current, target_center, positions, i, min_dist, priority)
            
            if blocking_bots:
                # Find alternative path around blocking bots
                intermediate_target = find_alternative_path(current, target_center, blocking_bots, min_dist)
                direction = intermediate_target - current
            else:
                # Direct path to target
                direction = target_center - current
            
            dist_to_target = np.linalg.norm(direction)
            if dist_to_target > 0.01:
                direction = direction / dist_to_target
            else:
                continue

        # Apply collision avoidance forces
        repulsion = np.zeros(2)
        must_yield = False

        for j, other_pos in enumerate(positions):
            if i != j:
                diff = current - other_pos
                dist = np.linalg.norm(diff)
                if dist < min_dist and dist > 0:
                    # Yield if other bot has higher priority (lower priority value)
                    if priority[j] < priority[i]:
                        must_yield = True
                    # Add repulsion force
                    repulsion_strength = (1 - dist / min_dist) * 1.5
                    repulsion += (diff / dist) * repulsion_strength

        if not in_target:  # Only apply yielding behavior outside target region
            if must_yield:
                # Yield by moving away or slowing down significantly
                move_vec = repulsion
                if np.linalg.norm(move_vec) > 0:
                    move_vec = move_vec / np.linalg.norm(move_vec) * (speed * 0.3)
                else:
                    move_vec = np.zeros(2)
            else:
                # Normal movement with collision avoidance
                if 'direction' in locals():
                    move_vec = direction + repulsion * 0.4
                    if np.linalg.norm(move_vec) > 0:
                        move_vec = move_vec / np.linalg.norm(move_vec) * speed
                    else:
                        move_vec = np.zeros(2)
        else:
            # Inside target region - gentler collision avoidance
            if 'direction' in locals():
                move_vec = direction + repulsion * 0.6
                if np.linalg.norm(move_vec) > 0:
                    move_vec = move_vec / np.linalg.norm(move_vec) * (speed * 0.4)

        # Apply movement
        new_pos = current + move_vec
        
        # Ensure bots don't leave target region if they're already inside
        if in_target:
            if not is_in_target_region(new_pos, target_center, target_radius):
                # Clamp position to target boundary
                to_center = new_pos - target_center
                to_center_dist = np.linalg.norm(to_center)
                if to_center_dist > target_radius:
                    new_pos = target_center + (to_center / to_center_dist) * target_radius
        
        p.resetBasePositionAndOrientation(bot_id, [new_pos[0], new_pos[1], bot_height / 2], [0, 0, 0, 1])

# Simulation loop
print("Enhanced Multi-Bot Simulation with Pathfinding!")
print("Click in the PyBullet window to set new target locations!")
print("Features:")
print("- Bots can enter and stay in target region")
print("- Dynamic pathfinding around blocking bots")
print("- Optimized spacing within target region")

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
    
    # Move bots with enhanced pathfinding
    move_bots_with_pathfinding(bot_ids, target_center, target_radius, speed, min_dist)
    
    # Step simulation
    p.stepSimulation()
    time.sleep(1. / 240.)
    
    # Print status every 200 steps
    if step % 200 == 0:
        positions = get_bot_positions(bot_ids)
        bots_in_target = sum(1 for pos in positions if is_in_target_region(pos, target_center, target_radius))
        print(f"Step {step}: {bots_in_target}/{num_bots} bots in target region")

p.disconnect()
