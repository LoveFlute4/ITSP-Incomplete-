import cv2
import numpy as np
import cv2.aruco as aruco
import time
import requests
from requests.exceptions import RequestException
import socket
import subprocess
import platform
from collections import deque

# CONFIG
BOT_IDS = [1]
TARGET_ID = 42  # Fixed: was [42], should be int
ESP32_IP = None  # Auto-detected
MANUAL_ESP32_IP = "192.168.68.145"  # Updated before every trial
COMMAND_DELAY = 0.1  # Minimum delay between ESP commands in seconds

# TRACKING CONFIG
MAX_MISSING_FRAMES = 5  # Max frames a marker can be missing before stopping
POSITION_HISTORY_SIZE = 5  # For velocity calculation
MIN_DISTANCE_THRESHOLD = 20  # Distance at which to stop
ANGLE_THRESHOLD = 6  # Max angle for alignment

# Phone Camera Setting
PHONE_IP = "192.168.68.106"  # Update before every trial
PHONE_PORT = 8080  # Port for IP Camera

# Global variables for command tracking
last_command_time = 0
current_command = None

# CONNECTING TO ESP32
def find_esp32_ip():
    global ESP32_IP
    
    print("Searching for ESP32 on network...")

    # Method 1: Try the Manual IP (Works mostly)
    if test_esp32_connection(MANUAL_ESP32_IP):
        print(f"Found ESP32 at backup IP: {MANUAL_ESP32_IP}")
        return MANUAL_ESP32_IP
    
    # Method 2: Manual input
    print("\nESP32 IP discovery failed.")
    print("Please check your ESP32's Serial Monitor for its IP address.")
    print("Look for the 'ROBOT CAR NETWORK INFORMATION' section.")
    manual_ip = input("Enter ESP32 IP address manually: ").strip()

    if manual_ip and test_esp32_connection(manual_ip):
        print(f"Connection successful! Updating MANUAL_ESP32_IP to '{manual_ip}' in the code")
        return manual_ip
    
    return None 

def test_esp32_connection(ip_address):
    try:
        response = requests.get(f"http://{ip_address}/", timeout=2)
        return response.status_code == 200
    except:
        return False

def get_phone_url():
    possible_urls = [
        f"http://{PHONE_IP}:{PHONE_PORT}/videofeed",  # IP Webcam
        f"http://{PHONE_IP}:{PHONE_PORT}/video",      # DroidCam
        f"http://{PHONE_IP}:{PHONE_PORT}/mjpeg",      # Other apps
        f"http://{PHONE_IP}:{PHONE_PORT}/cam/1/stream.mjpeg"  # Alternative
    ]
    print("Testing phone camera connections...")
    for url in possible_urls:
        try:
            cap = cv2.VideoCapture(url)
            ret, frame = cap.read()
            cap.release()
            if ret and frame is not None:
                print(f"Camera found at: {url}")
                return url
        except:
            continue
    
    print("Auto-detection failed. Please enter your phone's camera stream URL:")
    print("Common formats:")
    for url in possible_urls:
        print(f" - {url}")
    manual_url = input("Enter camera URL: ").strip()
    return manual_url if manual_url else possible_urls[0]

# CAMERA SETUP
def setup_camera():
    camera_url = get_phone_url()
    print(f"Connecting to camera: {camera_url}")

    cap = cv2.VideoCapture(camera_url, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer
    cap.set(cv2.CAP_PROP_FPS, 30)
    # Additional settings for better real-time performance
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Test connection
    ret, frame = cap.read()
    if not ret:
        print("Failed to connect to camera!")
        return None 
    print("Camera connected successfully!")
    return cap

def send_command(command):
    global last_command_time, current_command, ESP32_IP
    
    print(f"\n[DEBUG] Attempting to send: {command}")
    print(f"[DEBUG] Last command: {current_command}")
    
    if ESP32_IP is None:
        print("ESP32 IP not set!")
        return False
    
    if command == current_command:
        print("[DEBUG] Same as last command - skipping")
        return True
    
    if time.time() - last_command_time < COMMAND_DELAY:
        print(f"[DEBUG] Too soon since last command (delay: {COMMAND_DELAY}s)")
        return True
    
    try:
        url = f"http://{ESP32_IP}/{command.lower()}"
        print(f"[DEBUG] Sending to: {url}")
        
        response = requests.get(url, timeout=1.0)
        print(f"[DEBUG] Response: {response.status_code}")
        
        if response.status_code == 200:
            print(f"✓ Command '{command}' sent successfully")
            current_command = command
            last_command_time = time.time()
            return True
        else:
            print(f"✗ Command '{command}' failed with status {response.status_code}")
            return False
            
    except RequestException as e:
        print(f"✗ Failed to send command '{command}': {str(e)}")
        return False

# Navigation Calculations
def calculate_navigation(bot_center, bot_dir, target_pos):
    if target_pos is None or bot_center is None or bot_dir is None:
        return 0, 0, "STOP"
    
    bt_vec = target_pos - bot_center
    distance = np.linalg.norm(bt_vec)

    if distance < MIN_DISTANCE_THRESHOLD:
        return 0, distance, "STOP"
    
    bt_norm = bt_vec / distance
    bot_dir_norm = bot_dir / np.linalg.norm(bot_dir)

    # Signed Angle (degrees)
    angle = np.degrees(np.arctan2(
        bot_dir_norm[0]*bt_norm[1] - bot_dir_norm[1]*bt_norm[0],
        bot_dir_norm[0]*bt_norm[0] + bot_dir_norm[1]*bt_norm[1]
    ))

    # Generate command based on angle and distance
    if abs(angle) < ANGLE_THRESHOLD:
        command = "FORWARD"
    else: 
        direction = "RIGHT" if angle > 0 else "LEFT"
        command = direction  # Simplified for ESP32 commands
    
    return angle, distance, command

# GET BOT POSE
def get_bot_pose(corners):
    c = corners[0]
    center = np.mean(c, axis=0)
    front = (c[0] + c[1]) / 2  # Assume top edge is front
    direction = front - center
    return center, direction

# PREDICTION BASED ON VELOCITY
def predict_position(position_history, frames_ahead=1):
    if len(position_history) < 2:
        return position_history[-1] if position_history else None
    
    # Calculate velocity from last two positions
    velocity = position_history[-1] - position_history[-2]
    predicted_pos = position_history[-1] + velocity * frames_ahead
    return predicted_pos

# TRACKER CLASS
class MarkerTracker:
    def __init__(self, marker_id):
        self.marker_id = marker_id
        self.position_history = deque(maxlen=POSITION_HISTORY_SIZE)
        self.direction_history = deque(maxlen=3)
        self.missing_frames = 0
        self.last_command = "STOP"
        self.last_seen_time = time.time()
        
    def update(self, center, direction):
        self.position_history.append(center)
        self.direction_history.append(direction)
        self.missing_frames = 0
        self.last_seen_time = time.time()
        
    def increment_missing(self):
        self.missing_frames += 1
        
    def is_active(self):
        return self.missing_frames < MAX_MISSING_FRAMES
    
    def get_smoothed_position(self):
        if len(self.position_history) == 0:
            return None
        # Return last known position if not enough history
        if len(self.position_history) < 3:
            return self.position_history[-1]
        # Use recent average for smoother tracking
        recent_positions = list(self.position_history)[-3:]
        return np.mean(recent_positions, axis=0)
    
    def get_smoothed_direction(self):
        if len(self.direction_history) == 0:
            return None
        # Use recent average for smoother direction
        recent_directions = list(self.direction_history)[-2:]
        return np.mean(recent_directions, axis=0)

# MAIN FUNCTION
def main():
    global ESP32_IP
    frame_skip = 5 # Number of frames we will skip in between 
    
    # Find ESP32 on network
    ESP32_IP = find_esp32_ip()
    if ESP32_IP is None:
        print("Could not find ESP32. Please check your setup.")
        return
    
    # Setup Camera
    cap = setup_camera()
    if cap is None:
        print("Camera Setup Failed!")
        return
    
    # ArUco Detection
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    detector = aruco.ArucoDetector(aruco_dict)
    prev_time = time.time()

    # Initialize trackers
    bot_trackers = {bot_id: MarkerTracker(bot_id) for bot_id in BOT_IDS}
    target_tracker = MarkerTracker(TARGET_ID)

    # Performance monitoring
    frame_count = 0
    processing_times = deque(maxlen=30)
    
    print(f"\n=== Starting Navigation System ===")
    print(f"Target ID: {TARGET_ID}")
    print(f"Bot IDs: {BOT_IDS}")
    print(f"ESP32 IP: {ESP32_IP}")

    try:
        while True:
            start_time = time.time()

            # Clear buffer by reading multiple frames quickly
            for _ in range(3):
                ret, frame = cap.read()
                if not ret:
                    break
        
            if not ret:
                print("Failed to read frame")
                break
            
            # Process frame
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            
            if ids is not None:
                print(f"Detected IDs: {ids.flatten()}")
            else:
                print("No markers detected")

            # Reset all trackers to missing
            for tracker in bot_trackers.values():
                tracker.increment_missing()
            target_tracker.increment_missing()
            
            # Update trackers with detected markers
            detected_markers = {}
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    marker_id = int(marker_id)
                    center, direction = get_bot_pose(corners[i])
                    detected_markers[marker_id] = (center, direction)
                
                    # Update appropriate tracker
                    if marker_id == TARGET_ID:
                        target_tracker.update(center, direction)
                    elif marker_id in BOT_IDS:
                        bot_trackers[marker_id].update(center, direction)
            
            # Process navigation only if target is active and every {frame_skip} frames
            if target_tracker.is_active() && frame_count % frame_skip == 0:
                target_pos = target_tracker.get_smoothed_position()
                print(f"[TARGET] Active | Position: {target_pos}")
                
                # Only proceed if we have a valid target position
                if target_pos is not None:
                    print(f"[NAV] Target valid - proceeding with bots")
                    
                    # Use prediction if target is moving
                    if len(target_tracker.position_history) >= 2:
                        predicted_target = predict_position(target_tracker.position_history, frames_ahead=2)
                        if predicted_target is not None:
                            target_pos = predicted_target

                    for bot_id in BOT_IDS:
                        tracker = bot_trackers[bot_id]

                        if tracker.is_active():
                            bot_pos = tracker.get_smoothed_position()
                            bot_dir = tracker.get_smoothed_direction()

                            if bot_pos is not None and bot_dir is not None:
                                try:
                                    angle, distance, command = calculate_navigation(bot_pos, bot_dir, target_pos)
                                    tracker.last_command = command
                                    send_command(command)

                                    # Visual feedback
                                    cv2.arrowedLine(frame,
                                                  tuple(bot_pos.astype(int)),
                                                  tuple((bot_pos + bot_dir * 40).astype(int)),
                                                  (0, 255, 0), 2)

                                    cv2.arrowedLine(frame,
                                                  tuple(bot_pos.astype(int)),
                                                  tuple(target_pos.astype(int)),
                                                  (0, 0, 255), 2)

                                    # Status text
                                    x, y = int(bot_pos[0]), int(bot_pos[1])
                                    status_color = (0, 255, 0) if tracker.missing_frames == 0 else (0, 255, 255)
                                    cv2.putText(frame, f"Bot {bot_id}: {command}", (x, y - 10),
                                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
                                    cv2.putText(frame, f"Dist: {distance:.1f} | Angle: {angle:.1f}°", (x, y + 15),
                                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, status_color, 1)
                                    
                                except Exception as e:
                                    print(f"Navigation error: {str(e)}")
                                    tracker.last_command = "STOP"
                                    send_command("STOP")
                            else:
                                print(f"[BOT {bot_id}] Invalid position or direction data")
                        else:
                            # Bot not detected - show last known command
                            cv2.putText(frame, f"Bot {bot_id}: LOST - {tracker.last_command}", (10, 60 + bot_id * 20),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                            if tracker.last_command != "STOP":
                                send_command("STOP")
                                tracker.last_command = "STOP"
                else:
                    print("[NAV] Target position is None!")
            elif frame_count % frame_skip != 0:
                continue
            else:
                print("[TARGET] Inactive or lost")
                # Target not detected - stop all bots
                cv2.putText(frame, "TARGET LOST - STOPPING ALL BOTS", (10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                for tracker in bot_trackers.values():
                    if tracker.last_command != "STOP":
                        send_command("STOP")
                        tracker.last_command = "STOP"

            # Performance info
            processing_time = time.time() - start_time
            processing_times.append(processing_time)
        
            fps = 1 / (time.time() - prev_time)
            prev_time = time.time()
            avg_processing_time = np.mean(processing_times) * 1000  # Convert to ms
        
            cv2.putText(frame, f"FPS: {fps:.1f} | Processing: {avg_processing_time:.1f}ms", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # Show detection count
            detection_count = len(detected_markers)
            cv2.putText(frame, f"Detected: {detection_count} markers", (10, frame.shape[0] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
            cv2.imshow("Robot Navigation System", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            frame_count += 1

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Send final stop command
        send_command("STOP")
        cap.release()
        cv2.destroyAllWindows()

        # Print performance stats
        if processing_times:
            print(f"\nPerformance Summary:")
            print(f"Average processing time: {np.mean(processing_times)*1000:.2f}ms")
            print(f"Total frames processed: {frame_count}")

if __name__ == "__main__":
    print("=== Robot Navigation System ===")
    print("Make sure all devices are connected to 'Tinkerers' Lab' WiFi")
    print("1. ESP32 should be running and connected")
    print("2. Phone should be running IP camera app")
    print("3. Laptop should be on same network")
    print()
    
    main()
