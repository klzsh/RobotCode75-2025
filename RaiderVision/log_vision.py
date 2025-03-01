
import cv2
import numpy as np

DilationKernel = 2
DilationIterations = 4

# URL of the MJPG stream
stream_url = "http://photon-frontcams.local:1184/stream.mjpg"

# Open the video stream
cap = cv2.VideoCapture(stream_url)

show_cap = False

def getXFromCenter(bboxes, frameWidth):
    # sort by highest height, lowest widht, distance to center, highest area
    min_area = 0
    min_height_width_ratio = 0

    bboxes = [bbox for bbox in bboxes if bbox[2] * bbox[3] > min_area]
    bboxes = [bbox for bbox in bboxes if bbox[3] / bbox[2] > min_height_width_ratio]

    sorted_vals = sorted(bboxes, key=lambda x: abs(x[0] + x[2] / 2 - frameWidth / 2))

    # TODO draw this bounding box for debugging
    
    return sorted_vals[0][0] + sorted_vals[0][2] / 2 - frameWidth / 2
    

MIN_CONTOUR_AREA = 500

# Create a window for sliders
cv2.namedWindow("Controls")

while True:
    # Read frame from stream
    ret, frame = cap.read()
    if not ret:
        print("Error: Unable to read frame")
        break

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # hsv = cv2.blur(hsv, (3, 3))

    # Thresholding the HSV image
    lower_bound = np.array([0,0,0])#np.array([HLower, SLower, VLower])
    upper_bound = np.array([255,255,0])#np.array([HUpper, SUpper, VUpper])
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # mask = cv2.inRange(hsv, 127, 255)

    # Apply dilation
    kernel = np.ones((DilationKernel, DilationKernel), np.uint8)
    dilated_mask = cv2.dilate(mask, kernel, iterations=DilationIterations)

    contours, hierarchy = cv2.findContours(dilated_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    all_contours = cv2.drawContours(dilated_mask, [cnt for cnt in contours if cv2.contourArea(cnt) > MIN_CONTOUR_AREA], -1, (0,255,0), 3)

    bboxes = []
    for cnt in contours:
        if cv2.contourArea(cnt) < MIN_CONTOUR_AREA:
            continue
        x,y,w,h = cv2.boundingRect(cnt)
        cv2.rectangle(hsv,(x,y),(x+w,y+h),(0,255,0),2)

    # contourImg = cv2.cvtColor(countorImg, cv2.COLOR_HSV2BGR)

    # Show original and processed frames
    cv2.imshow("Live Stream", hsv)
    cv2.imshow("All Contours", mask)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
