from ultralytics import YOLO
import cv2
import requests

# Load pretrained YOLO model (COCO dataset)
model = YOLO("yolov8n.pt")
# model.to("cuda") # enable GPU usage, oh wait im using AMD

# ESP32-CAM stream
esp_ip = "192.168.1.41"
url = f"http://{esp_ip}:81/stream"

# Configure stream
requests.get(f"http://{esp_ip}/control?var=framesize&val=10") # resolution; val 10 = 640x480, 6 = 320x240
requests.get(f"http://{esp_ip}/control?var=quality&val=12")   # quality; 4-63 (lower is better, better is lower fps)
requests.get(f"http://{esp_ip}/control?var=brightness&val=0") # brightness; -3 to 3
requests.get(f"http://{esp_ip}/control?var=contrast&val=0")   # contrast; -3 to 3
requests.get(f"http://{esp_ip}/control?var=saturation&val=0") # saturation; -4 to 4

# Get video stream
cap = cv2.VideoCapture(url)

# Detection loop
if not cap.isOpened():
    print("Failed to open stream")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame grab failed")
        break

    # Run YOLO inference
    frame_small = cv2.resize(frame, (320, 240)) # resize before passing into model
    results = model(frame_small, imgsz=320)     # small frame = faster

    # results = model(frame, imgsz=320,classes=[0])
    
    # Draw bounding boxes
    for box in results[0].boxes:
        x1, y1, x2, y2 = box.xyxy[0] # get bounding box
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to integers

        # Compute frame center
        xc = int((x1 + x2) / 2)
        yc = int((y1 + y2) / 2)

        # Draw detections
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)  # bounding box
        cv2.circle(frame, (xc, yc), 5, (0,0,255), -1)           # center of box
        
        # print(f"Center: ({xc}, {yc})") # debugging

        # Compute stream center
        h, w, _ = frame.shape
        x_center_img = w / 2
        y_center_img = h / 2

        # Compute & normalize frame-stream error
        error_x = (xc - x_center_img) / (w / 2)
        error_y = (yc - y_center_img) / (h / 2)

        print(f"Normalized error: ({error_x:.2f}, {error_y:.2f})")

    cv2.imshow("Tracking", frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()