import cv2
import numpy as np
import cv2.aruco as aruco
import time
import requests
from requests.exceptions import RequestException
import socket
import subprocess
import platform

# CONFIG
BOT_IDS = [1, 2, 3]
TARGET_ID = 42
ESP32_IP = None  # Will be auto-discovered or set manually
ESP32_HOSTNAME = "robotcar.local"  # mDNS hostname
COMMAND_DELAY = 0.1  # Minimum delay between commands in seconds

# Phone camera settings - update these for your phone's IP camera app
PHONE_IP = "192.168.1.100"  # Replace with your phone's IP on Tinkerers' Lab network
PHONE_PORT = 8080  # Common port for IP camera apps

def find_esp32_ip():
    """Try to find ESP32 IP address using multiple methods"""
    global ESP32_IP
    
    print("Searching for ESP32 on network...")
    
    # Method 1: Try mDNS hostname resolution
    try:
        import socket
        esp32_ip = socket.gethostbyname(ESP32_HOSTNAME)
        print(f"Found ESP32 via mDNS: {esp32_ip}")
        return esp32_ip
    except:
        print("mDNS resolution failed")
    
    # Method 2: Network scan (basic ping sweep)
    # Get laptop's IP to determine network range
    try:
        hostname = socket.gethostname()
        laptop_ip = socket.gethostbyname(hostname)
        network_base = '.'.join(laptop_ip.split('.')[:-1])
        
        print(f"Scanning network {network_base}.x...")
        
        # Scan common IP ranges
        for i in range(100, 200):  # Scan .100 to .199
            test_ip = f"{network_base}.{i}"
            if test_esp32_connection(test_ip):
                print(f"Found ESP32 at: {test_ip}")
                return test_ip
                
    except Exception as e:
        print(f"Network scan failed: {e}")
    
    # Method 3: Manual input
    print("Automatic discovery failed.")
    print("Please check your ESP32's Serial Monitor for its IP address.")
    manual_ip = input("Enter ESP32 IP address manually: ").strip()
    
    if manual_ip and test_esp32_connection(manual_ip):
        return manual_ip
    
    return None

def test_esp32_connection(ip_address):
    """Test if ESP32 is accessible at given IP"""
    try:
        response = requests.get(f"http://{ip_address}/", timeout=2)
        return response.status_code == 200
    except:
        return False

def get_phone_camera_url():
    """Get the correct camera URL for your phone"""
    # Common IP camera app URLs:
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
    
    # If auto-detection fails, ask user
    print("Auto-detection failed. Please enter your phone's camera stream URL:")
    print("Common formats:")
    print(f"  - http://{PHONE_IP}:{PHONE_PORT}/videofeed")
    print(f"  - http://{PHONE_IP}:{PHONE_PORT}/video")
    
    manual_url = input("Enter camera URL: ").strip()
    return manual_url if manual_url else possible_urls[0]

# CAMERA SETUP
def setup_camera():
    camera_url = get_phone_camera_url()
    print(f"Connecting to camera: {camera_url}")
    
    cap = cv2.VideoCapture(camera_url, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # Test the connection
    ret, frame = cap.read()
    if not ret:
        print("Failed to connect to camera!")
        return None
    
    print("Camera connected successfully!")
    return cap

# Wifi Command Function with Error Handling
last_command_time = time.time()
current_command = None

def send_command(command):
    global last_command_time, current_command, ESP32_IP
    
    if ESP32_IP is None:
        print("ESP32 IP not set!")
        return False
    
    # Don't send the same command repeatedly
    if command == current_command:
        return True
    
    # Enforce minimum delay between commands
    if time.time() - last_command_time < COMMAND_DELAY:
        return True
    
    try:
        url = f"http://{ESP32_IP}/{command.lower()}"
        response = requests.get(url, timeout=1.0)
        
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

# Navigation Calculations (unchanged)
def calculate_navigation(bot_center, bot_dir, target_pos):
    bt_vec = target_pos - bot_center
    bt_norm = bt_vec / np.linalg.norm(bt_vec)
    bot_dir_norm = bot_dir / np.linalg.norm(bot_dir)

    angle = np.degrees(np.arctan2(
        bot_dir_norm[0]*bt_norm[1] - bot_dir_norm[1]*bt_norm[0],
        bot_dir_norm[0]*bt_norm[0] + bot_dir_norm[1]*bt_norm[1]
    ))
    distance = np.linalg.norm(bt_vec)
    return angle, distance

def get_bot_pose(corners):
    c = corners[0]
    center = np.mean(c, axis=0)
    front = (c[0] + c[1]) / 2
    direction = front - center
    return center, direction

def display_connection_status(frame):
    """Display connection status on frame"""
    status_text = f"ESP32: {ESP32_IP if ESP32_IP else 'DISCONNECTED'}"
    cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

# MAIN
def main():
    global ESP32_IP
    
    # Find ESP32 on network
    ESP32_IP = find_esp32_ip()
    if ESP32_IP is None:
        print("Could not find ESP32. Please check your setup.")
        return
    
    # Setup camera
    cap = setup_camera()
    if cap is None:
        print("Camera setup failed!")
        return
    
    # Setup ArUco detection
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    detector = aruco.ArucoDetector(aruco_dict)
    
    # Clear camera buffer
    for _ in range(5): 
        cap.read()
    
    print("Starting navigation system...")
    print("Press 'q' to quit, 's' to stop robot")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to get frame")
                break

            # Display connection status
            display_connection_status(frame)

            # Process at reduced resolution for speed
            small_frame = cv2.resize(frame, (320, 240))
            gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            markers = {}
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    markers[int(marker_id)] = corners[i]

            if TARGET_ID in markers:
                target_pos, _ = get_bot_pose(markers[TARGET_ID])
                target_pos *= 2  # Scale back up for display

                for bot_id in BOT_IDS:
                    if bot_id in markers:
                        bot_pos, bot_dir = get_bot_pose(markers[bot_id])
                        bot_pos *= 2  # Scale back up for display
                        bot_dir *= 2
                        
                        angle, distance = calculate_navigation(bot_pos, bot_dir, target_pos)

                        # Visualization
                        cv2.arrowedLine(frame,
                                      tuple(bot_pos.astype(int)),
                                      tuple((bot_pos + bot_dir * 40).astype(int)),
                                      (0, 255, 0), 2)
                        cv2.arrowedLine(frame,
                                      tuple(bot_pos.astype(int)),
                                      tuple(target_pos.astype(int)),
                                      (0, 0, 255), 2)

                        # Determine and send command
                        if abs(angle) < 5:
                            cmd_text = f"FORWARD (dist: {distance:.1f})"
                            send_command("forward")
                        elif angle > 0:
                            cmd_text = f"RIGHT {abs(angle):.1f}°"
                            send_command("right")
                        else:
                            cmd_text = f"LEFT {abs(angle):.1f}°"
                            send_command("left")

                        # Display status
                        x, y = int(bot_pos[0]), int(bot_pos[1])
                        cv2.putText(frame, f"Bot {bot_id}: {cmd_text}", (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            cv2.imshow("Robot Navigation System", frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                send_command("stop")
                print("Manual stop command sent")

    except KeyboardInterrupt:
        print("\nShutting down...")
        
    finally:
        cap.release()
        cv2.destroyAllWindows()
        send_command("stop")  # Ensure robot stops when program ends
        print("Navigation system stopped")

if __name__ == "__main__":
    print("=== Robot Navigation System ===")
    print("Make sure all devices are connected to 'Tinkerers' Lab' WiFi")
    print("1. ESP32 should be running and connected")
    print("2. Phone should be running IP camera app")
    print("3. Laptop should be on same network")
    print()
    
    main()
