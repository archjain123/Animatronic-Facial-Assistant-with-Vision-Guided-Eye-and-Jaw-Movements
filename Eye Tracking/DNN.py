import cv2
import numpy as np
import socket
import time

# ---------------------------
# ESP32 CONFIG (UDP SEND)
# ---------------------------
ESP32_IP = "192.168.4.1"    # replace with your ESP32 IP
ESP32_PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ---------------------------
# LOAD DNN FACE DETECTOR
# ---------------------------
base_path = "/home/archit/Desktop/NOOR/"
modelFile = "res10_300x300_ssd_iter_140000.caffemodel"
configFile = "deploy.prototxt"
net = cv2.dnn.readNetFromCaffe(configFile, modelFile)

# ---------------------------
# WEBCAM
# ---------------------------
cap = cv2.VideoCapture(0)

# camera center (filled later)
cx_cam, cy_cam = 0, 0

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    h, w = frame.shape[:2]
    cx_cam, cy_cam = w // 2, h // 2

    # ---------------------------
    # DNN FOR FACE DETECTION
    # ---------------------------
    blob = cv2.dnn.blobFromImage(
        cv2.resize(frame, (300, 300)),
        1.0,
        (300, 300),
        (104.0, 177.0, 123.0)
    )
    net.setInput(blob)
    detections = net.forward()

    best_conf = 0
    face_center = None

    # find highest confidence detection
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]

        if confidence > 0.6 and confidence > best_conf:
            best_conf = confidence

            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (x1, y1, x2, y2) = box.astype("int")

            fx = (x1 + x2) // 2
            fy = (y1 + y2) // 2

            face_center = (fx, fy)

    # ---------------------------
    # IF FACE DETECTED: SEND DATA
    # ---------------------------
    if face_center:
        fx, fy = face_center

        # draw visualization
        cv2.circle(frame, (fx, fy), 5, (0, 255, 0), -1)
        cv2.circle(frame, (cx_cam, cy_cam), 5, (0, 0, 255), -1)
        cv2.line(frame, (cx_cam, cy_cam), (fx, fy), (255, 0, 0), 2)

        # Calculate offsets
        dx = fx - cx_cam     # positive → face is right of center
        dy = fy - cy_cam     # positive → face is below center

        # Scale down (avoid servo jerk)
        dx = int(dx / 10)
        dy = int(dy / 10)

        # FIX INVERSION HERE
        dx = -dx       # invert horizontal if needed
        dy = dy        # if vertical inverted, change to: dy = -dy

        # FORMAT: "dx,dy"
        message = f"{dx},{dy}"
        sock.sendto(message.encode(), (ESP32_IP, ESP32_PORT))

        cv2.putText(frame, message, (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

    cv2.imshow("Eye Tracker", frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
