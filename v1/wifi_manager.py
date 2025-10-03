"""
WiFi and Network Management Module
Handles ESP32 discovery, connection testing, and phone camera setup
"""

import requests
from requests.exceptions import RequestException
import cv2

class WiFiManager:
    def __init__(self, manual_esp32_ip="192.168.68.145", phone_ip="192.168.68.106", phone_port=8080):
        self.manual_esp32_ip = manual_esp32_ip
        self.phone_ip = phone_ip
        self.phone_port = phone_port
        self.esp32_ip = None
        
    def find_esp32_ip(self):
        """Find ESP32 IP address on network"""
        print("Searching for ESP32 on network...")

        # Method 1: Try the Manual IP (Works mostly)
        if self.test_esp32_connection(self.manual_esp32_ip):
            print(f"Found ESP32 at backup IP: {self.manual_esp32_ip}")
            self.esp32_ip = self.manual_esp32_ip
            return self.manual_esp32_ip
        
        # Method 2: Manual input
        print("\nESP32 IP discovery failed.")
        print("Please check your ESP32's Serial Monitor for its IP address.")
        print("Look for the 'ROBOT CAR NETWORK INFORMATION' section.")
        manual_ip = input("Enter ESP32 IP address manually: ").strip()

        if manual_ip and self.test_esp32_connection(manual_ip):
            print(f"Connection successful! Updating MANUAL_ESP32_IP to '{manual_ip}' in the code")
            self.esp32_ip = manual_ip
            return manual_ip
        
        return None 

    def test_esp32_connection(self, ip_address):
        """Test connection to ESP32 at given IP address"""
        try:
            response = requests.get(f"http://{ip_address}/", timeout=2)
            return response.status_code == 200
        except:
            return False

    def get_phone_url(self):
        """Get phone camera stream URL"""
        possible_urls = [
            f"http://{self.phone_ip}:{self.phone_port}/videofeed",  # IP Webcam
            f"http://{self.phone_ip}:{self.phone_port}/video",      # DroidCam
            f"http://{self.phone_ip}:{self.phone_port}/mjpeg",      # Other apps
            f"http://{self.phone_ip}:{self.phone_port}/cam/1/stream.mjpeg"  # Alternative
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

    def setup_camera(self):
        """Setup and configure camera connection"""
        camera_url = self.get_phone_url()
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

    def send_command(self, command):
        """Send command to ESP32"""
        if self.esp32_ip is None:
            print("ESP32 IP not set!")
            return False
        
        try:
            url = f"http://{self.esp32_ip}/{command.lower()}"
            print(f"[DEBUG] Sending to: {url}")
            
            response = requests.get(url, timeout=1.0)
            print(f"[DEBUG] Response: {response.status_code}")
            
            if response.status_code == 200:
                print(f"✓ Command '{command}' sent successfully")
                return True
            else:
                print(f"✗ Command '{command}' failed with status {response.status_code}")
                return False
                
        except RequestException as e:
            print(f"✗ Failed to send command '{command}': {str(e)}")
            return False
