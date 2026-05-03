from ultralytics import YOLO
import cv2
import requests
import serial
import time

# connect via serial
ser = serial.Serial("COM6", 115200, timeout=0)
last_read = 0

# Load pretrained YOLO model (COCO dataset)
# model = YOLO("yolov8n.pt")
model = YOLO("yolo26n.pt")
# model.to("cuda") # enable GPU usage, oh wait im using AMD

# ESP32-CAM stream
esp_ip = "192.168.1.57" # testing camera
# esp_ip = "192.168.1.41" # primary camera
url = f"http://{esp_ip}:81/stream"

# Configure stream
requests.get(f"http://{esp_ip}/control?var=framesize&val=7") # resolution; val 10 = 640x480, 7 = 320x240
requests.get(f"http://{esp_ip}/control?var=quality&val=12")   # quality; 4-63 (lower is better, better is lower fps)
requests.get(f"http://{esp_ip}/control?var=brightness&val=0") # brightness; -3 to 3
requests.get(f"http://{esp_ip}/control?var=contrast&val=0")   # contrast; -3 to 3
requests.get(f"http://{esp_ip}/control?var=saturation&val=0") # saturation; -4 to 4

# Get video stream
cap = cv2.VideoCapture(url)

if not cap.isOpened():
    print("Failed to open stream")
    exit()



# -----------------------------
# IoU FUNCTION
# -----------------------------
def iou(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    interW = max(0, xB - xA)
    interH = max(0, yB - yA)
    interArea = interW * interH

    areaA = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    areaB = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])

    return interArea / (areaA + areaB - interArea + 1e-6)


# -----------------------------
# MOUSE CALLBACK
# -----------------------------
click_x, click_y = -1, -1
selected_box = None
tracking_box = None
lock_active = False

def mouse(event, x, y, flags, param):
    global click_x, click_y, lock_active

    if event == cv2.EVENT_LBUTTONDOWN:
        click_x, click_y = x, y
        lock_active = False  # reset lock

    elif event == cv2.EVENT_RBUTTONDOWN:
        lock_active = False
        selected_box = None
        tracking_box = None
        click_x, click_y = -1, -1

cv2.namedWindow("Stream")
cv2.setMouseCallback("Stream", mouse)

# -----------------------------
# MAIN LOOP
# -----------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame grab failed")
        break

    # rotate stream
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # Run YOLO inference
    # frame_small = cv2.resize(frame, (320, 240)) # resize before passing into model
    # results = model(frame_small, imgsz=320, classes=[0], verbose=False)     # small frame = faster
    results = model(frame, imgsz=320,classes=[0], verbose=False)

    boxes = []
    for b in results[0].boxes:
        x1, y1, x2, y2 = map(int, b.xyxy[0])
        boxes.append((x1, y1, x2, y2))

    # STEP 1: if clicked, select closest detection
    if click_x != -1 and click_y != -1 and not lock_active:
        best_box = None
        best_dist = 1e9

        for b in boxes:
            cx = (b[0] + b[2]) // 2
            cy = (b[1] + b[3]) // 2

            dist = (cx - click_x) ** 2 + (cy - click_y) ** 2

            if dist < best_dist:
                best_dist = dist
                best_box = b

        if best_box is not None:
            selected_box = best_box
            tracking_box = best_box
            lock_active = True

    # STEP 2: tracking logic (keep matching same object)
    if lock_active and selected_box is not None:
        best_match = None
        best_score = 0

        for b in boxes:
            score = iou(b, selected_box)

            if score > best_score:
                best_score = score
                best_match = b

        # only update if match is strong
        if best_score > 0.2:
            tracking_box = best_match
            selected_box = best_match
        else:
            tracking_box = None  # lost target

    # DRAW RESULTS
    annotated = results[0].plot()

    # highlight tracked box
    if lock_active and tracking_box is not None:
        x1, y1, x2, y2 = tracking_box

        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 3)

        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        cv2.circle(annotated, (cx, cy), 6, (0, 0, 255), -1)
        
        # compute normalized error command signal
        h, w = frame.shape[:2]

        fx = w / 2  # frame width
        fy = h / 2  # frame height

        ex_px = cx - fx  # width error
        ey_px = cy - fy  # height error

        ex = ex_px / fx  # normalized width error
        ey = ey_px / fy  # normalized height error

        # clamping
        ex = max(-1.0, min(1.0, ex))
        ey = max(-1.0, min(1.0, ey))

        # add deadzone
        if abs(ex) < 0.05: ex = 0
        if abs(ey) < 0.05: ey = 0
    else:
        ex = 0
        ey = 0

    ser.write(f"{ex:.3f},{ey:.3f}\n".encode()) # send error to COM-connected ESP

    # play stream
    cv2.imshow("Stream", annotated)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()