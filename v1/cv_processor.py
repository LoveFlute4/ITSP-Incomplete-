"""
Computer Vision Processing Module
Handles ArUco marker detection, tracking, and navigation calculations
"""

import cv2
import numpy as np
import cv2.aruco as aruco
import time
from collections import deque

class CVProcessor:
    def __init__(self, bot_ids=[1], target_id=42):
        # Configuration
        self.bot_ids = bot_ids
        self.target_id = target_id
        
        # Tracking parameters
        self.max_missing_frames = 5
        self.position_history_size = 5
        self.min_distance_threshold = 20
        self.angle_threshold = 6
        
        # ArUco setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.detector = aruco.ArucoDetector(self.aruco_dict)
        
        # Initialize trackers
        self.bot_trackers = {bot_id: MarkerTracker(bot_id, self.position_history_size, self.max_missing_frames) 
                           for bot_id in self.bot_ids}
        self.target_tracker = MarkerTracker(self.target_id, self.position_history_size, self.max_missing_frames)
        
        # Performance monitoring
        self.processing_times = deque(maxlen=30)
        
    def detect_markers(self, frame):
        """Detect ArUco markers in frame"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        
        detected_markers = {}
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                marker_id = int(marker_id)
                center, direction = self.get_bot_pose(corners[i])
                detected_markers[marker_id] = (center, direction)
        
        return detected_markers
    
    def get_bot_pose(self, corners):
        """Calculate bot center and direction from marker corners"""
        c = corners[0]
        center = np.mean(c, axis=0)
        front = (c[0] + c[1]) / 2  # Assume top edge is front
        direction = front - center
        return center, direction
    
    def update_trackers(self, detected_markers):
        """Update all trackers with detected markers"""
        # Reset all trackers to missing
        for tracker in self.bot_trackers.values():
            tracker.increment_missing()
        self.target_tracker.increment_missing()
        
        # Update trackers with detected markers
        for marker_id, (center, direction) in detected_markers.items():
            if marker_id == self.target_id:
                self.target_tracker.update(center, direction)
            elif marker_id in self.bot_ids:
                self.bot_trackers[marker_id].update(center, direction)
    
    def calculate_navigation(self, bot_center, bot_dir, target_pos):
        """Calculate navigation command based on bot and target positions"""
        if target_pos is None or bot_center is None or bot_dir is None:
            return 0, 0, "STOP"
        
        bt_vec = target_pos - bot_center
        distance = np.linalg.norm(bt_vec)

        if distance < self.min_distance_threshold:
            return 0, distance, "STOP"
        
        bt_norm = bt_vec / distance
        bot_dir_norm = bot_dir / np.linalg.norm(bot_dir)

        # Signed Angle (degrees)
        angle = np.degrees(np.arctan2(
            bot_dir_norm[0]*bt_norm[1] - bot_dir_norm[1]*bt_norm[0],
            bot_dir_norm[0]*bt_norm[0] + bot_dir_norm[1]*bt_norm[1]
        ))

        # Generate command based on angle and distance
        if abs(angle) < self.angle_threshold:
            command = "FORWARD"
        else: 
            direction = "RIGHT" if angle > 0 else "LEFT"
            command = direction
        
        return angle, distance, command
    
    def predict_position(self, position_history, frames_ahead=1):
        """Predict future position based on velocity"""
        if len(position_history) < 2:
            return position_history[-1] if position_history else None
        
        # Calculate velocity from last two positions
        velocity = position_history[-1] - position_history[-2]
        predicted_pos = position_history[-1] + velocity * frames_ahead
        return predicted_pos
    
    def process_navigation(self, frame, frame_count, frame_skip=5):
        """Process navigation for all bots"""
        navigation_commands = {}
        
        # Process navigation only if target is active and every {frame_skip} frames
        if self.target_tracker.is_active() and frame_count % frame_skip == 0:
            target_pos = self.target_tracker.get_smoothed_position()
            
            if target_pos is not None:
                # Use prediction if target is moving
                if len(self.target_tracker.position_history) >= 2:
                    predicted_target = self.predict_position(self.target_tracker.position_history, frames_ahead=2)
                    if predicted_target is not None:
                        target_pos = predicted_target

                for bot_id in self.bot_ids:
                    tracker = self.bot_trackers[bot_id]

                    if tracker.is_active():
                        bot_pos = tracker.get_smoothed_position()
                        bot_dir = tracker.get_smoothed_direction()

                        if bot_pos is not None and bot_dir is not None:
                            try:
                                angle, distance, command = self.calculate_navigation(bot_pos, bot_dir, target_pos)
                                tracker.last_command = command
                                navigation_commands[bot_id] = {
                                    'command': command,
                                    'angle': angle,
                                    'distance': distance,
                                    'bot_pos': bot_pos,
                                    'bot_dir': bot_dir,
                                    'target_pos': target_pos
                                }
                            except Exception as e:
                                print(f"Navigation error for bot {bot_id}: {str(e)}")
                                tracker.last_command = "STOP"
                                navigation_commands[bot_id] = {'command': "STOP"}
                    else:
                        # Bot not detected
                        if tracker.last_command != "STOP":
                            tracker.last_command = "STOP"
                            navigation_commands[bot_id] = {'command': "STOP"}
        elif frame_count % frame_skip == 0:
            # Target not detected - stop all bots
            for bot_id in self.bot_ids:
                tracker = self.bot_trackers[bot_id]
                if tracker.last_command != "STOP":
                    tracker.last_command = "STOP"
                    navigation_commands[bot_id] = {'command': "STOP"}
        
        return navigation_commands
    
    def draw_visualization(self, frame, navigation_commands, detected_markers):
        """Draw navigation visualization on frame"""
        # Draw visual feedback for navigation
        for bot_id, nav_data in navigation_commands.items():
            if 'bot_pos' in nav_data and 'bot_dir' in nav_data and 'target_pos' in nav_data:
                bot_pos = nav_data['bot_pos']
                bot_dir = nav_data['bot_dir']
                target_pos = nav_data['target_pos']
                
                # Draw bot direction arrow
                cv2.arrowedLine(frame,
                              tuple(bot_pos.astype(int)),
                              tuple((bot_pos + bot_dir * 40).astype(int)),
                              (0, 255, 0), 2)

                # Draw line to target
                cv2.arrowedLine(frame,
                              tuple(bot_pos.astype(int)),
                              tuple(target_pos.astype(int)),
                              (0, 0, 255), 2)

                # Status text
                x, y = int(bot_pos[0]), int(bot_pos[1])
                tracker = self.bot_trackers[bot_id]
                status_color = (0, 255, 0) if tracker.missing_frames == 0 else (0, 255, 255)
                cv2.putText(frame, f"Bot {bot_id}: {nav_data['command']}", (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
                if 'distance' in nav_data and 'angle' in nav_data:
                    cv2.putText(frame, f"Dist: {nav_data['distance']:.1f} | Angle: {nav_data['angle']:.1f}°", 
                               (x, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, status_color, 1)
        
        # Draw status for lost bots
        for bot_id in self.bot_ids:
            tracker = self.bot_trackers[bot_id]
            if not tracker.is_active():
                cv2.putText(frame, f"Bot {bot_id}: LOST - {tracker.last_command}", (10, 60 + bot_id * 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Draw target status
        if not self.target_tracker.is_active():
            cv2.putText(frame, "TARGET LOST - STOPPING ALL BOTS", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Show detection count
        detection_count = len(detected_markers)
        cv2.putText(frame, f"Detected: {detection_count} markers", (10, frame.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return frame


class MarkerTracker:
    def __init__(self, marker_id, position_history_size=5, max_missing_frames=5):
        self.marker_id = marker_id
        self.position_history = deque(maxlen=position_history_size)
        self.direction_history = deque(maxlen=3)
        self.missing_frames = 0
        self.max_missing_frames = max_missing_frames
        self.last_command = "STOP"
        self.last_seen_time = time.time()
        
    def update(self, center, direction):
        """Update tracker with new marker position"""
        self.position_history.append(center)
        self.direction_history.append(direction)
        self.missing_frames = 0
        self.last_seen_time = time.time()
        
    def increment_missing(self):
        """Increment missing frame counter"""
        self.missing_frames += 1
        
    def is_active(self):
        """Check if tracker is still active (not missing too many frames)"""
        return self.missing_frames < self.max_missing_frames
    
    def get_smoothed_position(self):
        """Get smoothed position based on recent history"""
        if len(self.position_history) == 0:
            return None
        # Return last known position if not enough history
        if len(self.position_history) < 3:
            return self.position_history[-1]
        # Use recent average for smoother tracking
        recent_positions = list(self.position_history)[-3:]
        return np.mean(recent_positions, axis=0)
    
    def get_smoothed_direction(self):
        """Get smoothed direction based on recent history"""
        if len(self.direction_history) == 0:
            return None
        # Use recent average for smoother direction
        recent_directions = list(self.direction_history)[-2:]
        return np.mean(recent_directions, axis=0)
