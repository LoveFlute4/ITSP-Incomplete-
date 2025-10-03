import cv2
import numpy as np
from pyzbar import pyzbar
import time

# Configuration
BOT_HEAD = "BOT-H-01"
BOT_BODY = "BOT-B-01"
TARGET = "TARGET-01"
MIN_QR_SIZE = 5  # pixels
WIDTH, HEIGHT = 960, 720  # Phone resolution

# Latency Optimization Setup
def setup_camera():
    cap = cv2.VideoCapture(
        "http://192.0.0.4:8080/videofeed?type=some.mjpeg&fps=30&resolution=960x720&noload=1",
        cv2.CAP_FFMPEG
    )
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimal buffer
    cap.set(cv2.CAP_PROP_FPS, 30)       # Match stream FPS
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
    return cap

def process_frame(frame):
    """Optimized QR detection with early filtering"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Fast adaptive thresholding for low-light
    gray = cv2.adaptiveThreshold(gray, 255, 
                               cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                               cv2.THRESH_BINARY, 11, 2)
    
    markers = {}
    for qr in pyzbar.decode(gray, symbols=[pyzbar.ZBarSymbol.QRCODE]):
        (x, y, w, h) = qr.rect
        if w >= MIN_QR_SIZE and h >= MIN_QR_SIZE:
            try:
                data = qr.data.decode('ascii')
                if "BOT-H" in data:
                    markers['head'] = np.array([x + w//2, y + h//2])
                elif "BOT-B" in data:
                    markers['body'] = np.array([x + w//2, y + h//2])
                elif "TARGET" in data:
                    markers['target'] = np.array([x + w//2, y + h//2])
            except:
                continue
    return markers

def calculate_navigation(head, body, target):
    """
    Returns (angle, distance)
    - angle: 0° when aligned, +ve when target is right, -ve when left
    - distance: in pixels
    """
    bh_vec = head - body  # Bot heading vector
    bt_vec = target - body  # Target vector
    
    # Normalize
    bh_norm = bh_vec / np.linalg.norm(bh_vec)
    bt_norm = bt_vec / np.linalg.norm(bt_vec)
    
    # Calculate angle with sign
    angle = np.degrees(np.arctan2(
        bh_norm[0]*bt_norm[1] - bh_norm[1]*bt_norm[0],  # Cross product
        bh_norm[0]*bt_norm[0] + bh_norm[1]*bt_norm[1]   # Dot product
    ))
    
    return angle, np.linalg.norm(bt_vec)

# Main Loop
cap = setup_camera()
prev_time = time.time()

while True:
    # Clear buffer
    for _ in range(2):
        ret, frame = cap.read()
    if not ret:
        break
    
    # Process frame (measure latency)
    start_process = time.time()
    markers = process_frame(frame)
    process_time = (time.time() - start_process) * 1000
    
    # Navigation logic
    if len(markers) == 3:
        angle, distance = calculate_navigation(
            markers['head'], 
            markers['body'], 
            markers['target']
        )
        
        # Draw navigation UI
        cv2.arrowedLine(frame, 
                       tuple(markers['body'].astype(int)),
                       tuple(markers['head'].astype(int)), 
                       (0, 255, 0), 3)  # Green: Bot direction
        
        cv2.arrowedLine(frame,
                       tuple(markers['body'].astype(int)),
                       tuple(markers['target'].astype(int)),
                       (0, 0, 255), 2)  # Red: Target direction
        
        # Display metrics
        fps = 1 / (time.time() - prev_time)
        prev_time = time.time()
        
        cv2.putText(frame, f"Angle: {angle:+.1f}°", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame, f"FPS: {fps:.1f} | Latency: {process_time:.1f}ms", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
        
        # Steering command
        if abs(angle) < 5:
            cmd = "ALIGNED"
            color = (0, 255, 0)
        else:
            direction = "RIGHT" if angle > 0 else "LEFT"
            cmd = f"TURN {direction} {abs(angle):.1f}°"
            color = (0, 165, 255)
            
        cv2.putText(frame, cmd, (10, 110),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
    
    # Display (comment out for absolute lowest latency)
    cv2.imshow("Navigation", frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
