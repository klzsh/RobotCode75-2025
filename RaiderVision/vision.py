
import cv2
import numpy as np
import ntcore

inst = ntcore.NetworkTableInstance.getDefault()

inst = ntcore.NetworkTableInstance.getDefault()
table = inst.getTable("RaiderVision")

inst.startClient4("RaiderVisionClient")
inst.setServerTeam('75') # where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar

DilationKernel = 2
DilationIterations = 4

# URL of the MJPG stream
stream_url = "http://photon-frontcams.local:1184/stream.mjpg"

# Open the video stream
cap = cv2.VideoCapture(stream_url)

MIN_CONTOUR_AREA = 500

while True:
    # Read frame from stream
    ret, frame = cap.read()
    if not ret:
        print("Error: Unable to read frame")
        break


    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_bound = np.array([0,0,0])
    upper_bound = np.array([255,255,0])
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    kernel = np.ones((DilationKernel, DilationKernel), np.uint8)
    dilated_mask = cv2.dilate(mask, kernel, iterations=DilationIterations)

    contours, hierarchy = cv2.findContours(dilated_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    min_area = 0
    min_height_width_ratio = 0
    bboxes = []
    for i, cnt in enumerate(contours):
        if cv2.contourArea(cnt) < MIN_CONTOUR_AREA:
            continue
        x,y,w,h = cv2.boundingRect(cnt)
        bboxes.append((x,y,w,h))
        
    bboxes = [bbox for bbox in bboxes if bbox[2] * bbox[3] > min_area]
    bboxes = [bbox for bbox in bboxes if bbox[3] / bbox[2] > min_height_width_ratio]

    sorted_vals = sorted(bboxes, key=lambda x: abs(x[0] + x[2] / 2 - frame.shape[1] / 2))
    if len(sorted_vals) > 0:
        x,y,w,h = sorted_vals[0]
        table.putString("target", f"{x},{y},{w},{h}")
    else:
        table.putString("target", f"")