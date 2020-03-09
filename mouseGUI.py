import cv2
import numpy as np
import csv
import time


# Create a file in 'log' directory and empty contents (if it already exists)
# Append timestamp in ms to name
location = "C:/Users/msrun/OneDrive - Imperial College London/Imperial/Fluidic Control/ControlSystem/logs"
start = time.time()
global prevMillis = 0
fileName = location + "/positions" + str(int(start*1000)) + ".csv"
fileName = 'test.csv' # For test purposes
with open(fileName, mode ='w') as posLog: 
    logger = csv.writer(posLog)
    logger.writerow(['Event', 'X', 'Y', 'Time / s'])


# Resolution in mm/pixel (Geomagic touch res ~0.055 mm)
resolution = 0.08
sideLength = 50
canvasX = int(sideLength/resolution)
canvasY = int(1.7321*(canvasX/2))
# Vertices of equilateral:
vt1 = [0, canvasY]
vt2 = [canvasX, canvasY]
vt3 = [canvasX/2, 0]

# Mouse callback function
def drawCables(event, x, y, flags, param):
    now = time.time()
    # Save time since beginning code in ms
    numMillis = now - start
    numMillis = int(numMillis*1000) # rounding error?
    timeDiff = numMillis - prevMillis
    prevMillis = numMillis
    with open(fileName, 'a') as posLog:
        logger = csv.writer(posLog)
        logger.writerow([event, x, y, numMillis, timeDiff])
    if event == cv2.EVENT_LBUTTONDOWN:
        cv2.circle(img, (x, y), 5, (255, 255, 255), 1)





# Lines below will be in controlSytem loop
# Call function to instantiate canvas and set callback function

# Create a black image, a window and bind the function to window
img = np.zeros((canvasX, canvasY, 3), np.uint8)
cv2.namedWindow('image')
cv2.setMouseCallback('image', drawCables)
while(1):
    cv2.imshow('image', img)
    if cv2.waitKey(20) & 0xFF == 27:
        break
cv2.destroyAllWindows()
