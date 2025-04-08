import cv2
from cvzone.FaceDetectionModule import FaceDetector
import pyfirmata
import numpy as np
import time

cap = cv2.VideoCapture(0)
ws, hs = 1280, 720
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("No se pudo acceder a la camara :c")
    exit()

port = "COM11"
board = pyfirmata.Arduino(port)
servo_pinX = board.get_pin('d:9:s') 
servo_pinY = board.get_pin('d:10:s') 
laser_pin = board.get_pin('d:8:o')  

detector = FaceDetector()
servoPos = [90, 90] 
last_blink_time = 0
blink_interval = 0.2 
laser_state = False

while True:
    success, img = cap.read()
    img, bboxs = detector.findFaces(img, draw=False)

    if bboxs:
        fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
        pos = [fx, fy]
        servoX = np.interp(fx, [0, ws], [180, 0])
        servoY = np.interp(fy, [0, hs], [180, 0])
        servoX = max(0, min(180, servoX))
        servoY = max(0, min(180, servoY))

        servoPos[0] = servoX
        servoPos[1] = servoY

        current_time = time.time()
        if current_time - last_blink_time > blink_interval:
            laser_state = not laser_state
            laser_pin.write(1 if laser_state else 0)
            last_blink_time = current_time

        cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
        cv2.putText(img, str(pos), (fx+15, fy-15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)  
        cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)  
        cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
        cv2.putText(img, "TARGET LOCKED", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
    
    else:
        laser_pin.write(0)  
        laser_state = False

        cv2.putText(img, "NO TARGET", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
        cv2.circle(img, (640, 360), 80, (0, 0, 255), 2)
        cv2.circle(img, (640, 360), 15, (0, 0, 255), cv2.FILLED)
        cv2.line(img, (0, 360), (ws, 360), (0, 0, 0), 2)  
        cv2.line(img, (640, hs), (640, 0), (0, 0, 0), 2)  

    cv2.putText(img, f'Servo X: {int(servoPos[0])} deg', (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    cv2.putText(img, f'Servo Y: {int(servoPos[1])} deg', (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

    servo_pinX.write(servoPos[0])
    servo_pinY.write(servoPos[1])

    cv2.imshow("Image", img)
    cv2.waitKey(1)

