import cv2

url = "http://192.168.1.41:81/stream"  # replace with your ESP32 IP

cap = cv2.VideoCapture(url)

if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Failed to grab frame")
        break

    cv2.imshow("ESP32 Stream", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
        break

cap.release()
cv2.destroyAllWindows()
