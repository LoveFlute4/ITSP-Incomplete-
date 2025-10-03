import cv2
import numpy as np
import cv2.aruco as aruco
import time

# CONFIG
BOT_IDS = [1, 2, 3]
TARGET_ID = 42

# CAMERA SETUP
def setup_camera():
    cap = cv2.VideoCapture("http://192.0.0.4:8080/videofeed", cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FPS, 30)
    return cap

# ANGLE + DISTANCE CALC
def calculate_navigation(bot_center, bot_dir, target_pos):
    bt_vec = target_pos - bot_center
    bt_norm = bt_vec / np.linalg.norm(bt_vec)
    bot_dir_norm = bot_dir / np.linalg.norm(bot_dir)

    # Signed angle (degrees)
    angle = np.degrees(np.arctan2(
        bot_dir_norm[0]*bt_norm[1] - bot_dir_norm[1]*bt_norm[0],
        bot_dir_norm[0]*bt_norm[0] + bot_dir_norm[1]*bt_norm[1]
    ))
    distance = np.linalg.norm(bt_vec)
    return angle, distance

# GET BOT POSE
def get_bot_pose(corners):
    c = corners[0]
    center = np.mean(c, axis=0)
    front = (c[0] + c[1]) / 2  # Assume top edge is front
    direction = front - center
    return center, direction

# MAIN
cap = setup_camera()
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
prev_time = time.time()

while True:
    for _ in range(2): ret, frame = cap.read()
    if not ret: break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

    markers = {}
    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            markers[int(marker_id)] = corners[i]

    if TARGET_ID in markers:
        target_pos, _ = get_bot_pose(markers[TARGET_ID])

        for bot_id in BOT_IDS:
            if bot_id in markers:
                bot_pos, bot_dir = get_bot_pose(markers[bot_id])
                angle, distance = calculate_navigation(bot_pos, bot_dir, target_pos)

                # Draw arrows
                cv2.arrowedLine(frame,
                                tuple(bot_pos.astype(int)),
                                tuple((bot_pos + bot_dir * 40).astype(int)),
                                (0, 255, 0), 2)

                cv2.arrowedLine(frame,
                                tuple(bot_pos.astype(int)),
                                tuple(target_pos.astype(int)),
                                (0, 0, 255), 2)

                # Annotate
                x, y = int(bot_pos[0]), int(bot_pos[1])
                cmd = "ALIGNED" if abs(angle) < 5 else f"TURN {'RIGHT' if angle > 0 else 'LEFT'} {abs(angle):.1f}°"
                cv2.putText(frame, f"Bot {bot_id}: {cmd}", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # FPS + Display
    fps = 1 / (time.time() - prev_time)
    prev_time = time.time()
    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    
    cv2.imshow("Aruco Navigation", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

