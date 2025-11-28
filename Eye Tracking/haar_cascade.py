# face_track_send.py
import cv2
import socket
import numpy as np
import time
import argparse

# ---------- USER CONFIG ----------
ESP32_IP = "192.168.1.23"   # <-- set to your ESP32 IP
ESP32_PORT = 4210

PAN_MIN = 40.0
PAN_MAX = 140.0
TILT_MIN = 45.0
TILT_MAX = 90.0

INVERT_PAN = False   # change if pan direction reversed
INVERT_TILT = False  # change if tilt direction reversed

CAM_INDEX = 0
# ----------------------------------

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise SystemExit("Camera not opened")

# load Haar cascade (grayscale face detector)
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

def map_normalized_to_range(norm, minv, maxv):
    # norm in [-1, +1]
    # by default left (-1) -> minv ; right(+1) -> maxv
    return minv + ( (norm + 1) / 2.0 ) * (maxv - minv)

last_send = 0
send_interval = 0.03  # seconds between UDP sends (30ms -> ~33Hz)

print("Starting camera. Press q to quit.")
while True:
    ret, frame = cap.read()
    if not ret:
        break
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(60,60))
    pan_angle = (PAN_MIN + PAN_MAX)/2.0
    tilt_angle = (TILT_MIN + TILT_MAX)/2.0

    if len(faces) > 0:
        # choose largest face (closest)
        faces = sorted(faces, key=lambda r: r[2]*r[3], reverse=True)
        (x,y,wf,hf) = faces[0]
        cx = x + wf/2.0
        cy = y + hf/2.0

        # draw box and center
        cv2.rectangle(frame, (x,y), (x+wf, y+hf), (0,255,0), 2)
        cv2.circle(frame, (int(cx), int(cy)), 4, (0,0,255), -1)

        # normalized coords -1..+1
        nx = (cx - w/2.0) / (w/2.0)
        ny = (cy - h/2.0) / (h/2.0)

        # For pan: we often invert depending on servo orientation,
        # if looking left should increase or decrease pan. Use INVERT_PAN to flip.
        if INVERT_PAN:
            nx = -nx
        if INVERT_TILT:
            ny = -ny

        # Map normalized to angle ranges
        pan_angle = map_normalized_to_range(nx, PAN_MIN, PAN_MAX)
        tilt_angle = map_normalized_to_range(ny, TILT_MIN, TILT_MAX)

        # clamp to safe range
        pan_angle = max(min(pan_angle, PAN_MAX), PAN_MIN)
        tilt_angle = max(min(tilt_angle, TILT_MAX), TILT_MIN)

        # overlay text
        cv2.putText(frame, f"PAN:{pan_angle:.1f} TILT:{tilt_angle:.1f}", (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

    # show image
    cv2.imshow("Face Track", frame)

    # send UDP at defined rate
    now = time.time()
    if now - last_send > send_interval:
        last_send = now
        msg = f"{pan_angle:.2f},{tilt_angle:.2f}\n".encode('utf-8')
        sock.sendto(msg, (ESP32_IP, ESP32_PORT))

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
sock.close()
