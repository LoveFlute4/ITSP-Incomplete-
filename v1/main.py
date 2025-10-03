"""
Main Robot Navigation System
Orchestrates WiFi management and computer vision processing
"""

import cv2
import numpy as np
import time
from collections import deque

from wifi_manager import WiFiManager
from cv_processor import CVProcessor

class RobotNavigationSystem:
    def __init__(self):
        # Configuration
        self.bot_ids = [1]
        self.target_id = 42
        self.manual_esp32_ip = "192.168.68.145"  # Update before every trial
        self.phone_ip = "192.168.68.106"  # Update before every trial
        self.phone_port = 8080
        
        # Command timing
        self.command_delay = 0.1  # Minimum delay between ESP commands in seconds
        self.last_command_time = 0
        self.current_command = None
        
        # Frame processing
        self.frame_skip = 5  # Number of frames to skip between processing
        self.frame_count = 0
        
        # Initialize components
        self.wifi_manager = WiFiManager(self.manual_esp32_ip, self.phone_ip, self.phone_port)
        self.cv_processor = CVProcessor(self.bot_ids, self.target_id)
        
        # Performance monitoring
        self.processing_times = deque(maxlen=30)
        
    def send_command_with_timing(self, command):
        """Send command with timing control"""
        if command == self.current_command:
            return True
        
        if time.time() - self.last_command_time < self.command_delay:
            return True
        
        success = self.wifi_manager.send_command(command)
        if success:
            self.current_command = command
            self.last_command_time = time.time()
        
        return success
    
    def initialize_system(self):
        """Initialize WiFi and camera connections"""
        print("=== Initializing Robot Navigation System ===")
        print("Make sure all devices are connected to 'Tinkerers' Lab' WiFi")
        print("1. ESP32 should be running and connected")
        print("2. Phone should be running IP camera app")
        print("3. Laptop should be on same network")
        print()
        
        # Find ESP32 on network
        if self.wifi_manager.find_esp32_ip() is None:
            print("Could not find ESP32. Please check your setup.")
            return False
        
        # Setup Camera
        self.cap = self.wifi_manager.setup_camera()
        if self.cap is None:
            print("Camera Setup Failed!")
            return False
        
        print(f"\n=== Starting Navigation System ===")
        print(f"Target ID: {self.target_id}")
        print(f"Bot IDs: {self.bot_ids}")
        print(f"ESP32 IP: {self.wifi_manager.esp32_ip}")
        
        return True
    
    def process_frame(self, frame):
        """Process a single frame"""
        start_time = time.time()
        
        # Detect markers
        detected_markers = self.cv_processor.detect_markers(frame)
        
        # Debug output
        if detected_markers:
            print(f"Detected IDs: {list(detected_markers.keys())}")
        else:
            print("No markers detected")
        
        # Update trackers
        self.cv_processor.update_trackers(detected_markers)
        
        # Process navigation
        navigation_commands = self.cv_processor.process_navigation(frame, self.frame_count, self.frame_skip)
        
        # Send commands to ESP32
        for bot_id, nav_data in navigation_commands.items():
            if 'command' in nav_data:
                self.send_command_with_timing(nav_data['command'])
        
        # Draw visualization
        frame = self.cv_processor.draw_visualization(frame, navigation_commands, detected_markers)
        
        # Performance monitoring
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)
        
        return frame, processing_time
    
    def run(self):
        """Main execution loop"""
        if not self.initialize_system():
            return
        
        prev_time = time.time()
        
        try:
            while True:
                # Clear buffer by reading multiple frames quickly
                for _ in range(3):
                    ret, frame = self.cap.read()
                    if not ret:
                        break
            
                if not ret:
                    print("Failed to read frame")
                    break
                
                # Process frame
                frame, processing_time = self.process_frame(frame)
                
                # Calculate and display performance metrics
                fps = 1 / (time.time() - prev_time)
                prev_time = time.time()
                avg_processing_time = np.mean(self.processing_times) * 1000  # Convert to ms
            
                cv2.putText(frame, f"FPS: {fps:.1f} | Processing: {avg_processing_time:.1f}ms", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
                # Display frame
                cv2.imshow("Robot Navigation System", frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                self.frame_count += 1

        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        # Send final stop command
        self.send_command_with_timing("STOP")
        
        # Release resources
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()

        # Print performance stats
        if self.processing_times:
            print(f"\nPerformance Summary:")
            print(f"Average processing time: {np.mean(self.processing_times)*1000:.2f}ms")
            print(f"Total frames processed: {self.frame_count}")


def main():
    """Main entry point"""
    navigation_system = RobotNavigationSystem()
    navigation_system.run()


if __name__ == "__main__":
    main()
