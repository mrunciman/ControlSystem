import cv2
import numpy as np
import csv
import time
import matplotlib.path as mpltPath


# Create a file in 'log' directory and empty contents (if it already exists)
# Append timestamp in ms to name
location = "C:/Users/msrun/OneDrive - Imperial College London/Imperial/Fluidic Control/ControlSystem/logs"
start = time.time()
prevMillis = 0
fileName = location + "/positions" + str(int(start*1000)) + ".csv"
# fileName = 'test.csv' # For test purposes
with open(fileName, mode ='w', newline='') as posLog: 
    logger = csv.writer(posLog)
    logger.writerow(['Event', 'X', 'Y', 'Timestamp', 'ms Since Last'])


# Resolution in mm/pixel (Geomagic touch res ~0.055 mm)
resolution = 0.06
sideLength = 50
canvasX = int(sideLength/resolution)
canvasY = int(1.7321*(canvasX/2))
xCoord = 0
yCoord = 0
# Vertices of equilateral:
vt1 = [0, canvasY]
vt2 = [canvasX, canvasY]
vt3 = [int(canvasX/2), 0]
vts = np.array([vt1, vt2, vt3])
path = mpltPath.Path(vts)
vts = vts.reshape((-1,1,2))
# A flag for when mouse is up or down
mouseDown = False
radius = 3
# neighbourhood = (x-2, y-2), (x+2, y-2), (x+2, y+2), (x-2, y+2)
# Check if position is within neighbourhood on down click
# If inside, move end effector and log.

# Mouse callback function
def drawCables(event, x, y, flags, param):
    global prevMillis
    global mouseDown
    global xCoord
    global yCoord
    now = time.time()
    # Save time since beginning code in ms
    numMillis = now - start
    numMillis = int(numMillis*1000) # rounding error?
    timeDiff = numMillis - prevMillis
    prevMillis = numMillis
    # Check if mouse button was pressed down (event 1)
    if event == 1:
        mouseDown = True
        img[:,:] = (255, 255, 255)
        cv2.polylines(img, [vts], True, (0, 0, 0), 1)
        cv2.circle(img, (x, y), radius, (0, 0, 0), -1)
        # print("Inside, Down")
    # Check if mouse button released (event 4) and redraw
    if event == 4:
        mouseDown = False
        # print("Inside, Up")
    # Check if point is inside triangle workspace
    inside = path.contains_point([x, y])
    if inside == True:
        if mouseDown == True:
            # print("Dooooon")
            if event == 0:
                xCoord = x#*resolution
                yCoord = (canvasY-y)#*resolution
                # print(xCoord, yCoord)
                with open(fileName, 'a', newline='') as posLog:
                    logger = csv.writer(posLog)
                    logger.writerow([event, xCoord, yCoord, now, timeDiff])
                    cv2.circle(img, (x, y), radius, (0, 0, 0), -1)
                    # img[:,:] = (255, 255, 255)
                    # cv2.polylines(img, [vts], True, (0, 0, 0), 1)
                    # cv2.line(img, (vt1[0], vt1[1]), (x, y), (0, 128, 0), 1)
                    # cv2.line(img, (vt2[0], vt2[1]), (x, y), (0, 128, 0), 1)
                    # cv2.line(img, (vt3[0], vt3[1]), (x, y), (0, 128, 0), 1)



# Lines below will be in controlSytem loop
# Call function to instantiate canvas and set callback function

# Create a black image, a window and bind the function to window
img = np.zeros(( canvasY+1, canvasX+1, 3), np.uint8)
img[:,:] = (255, 255, 255)
cv2.polylines(img, [vts], True, (0, 0, 0), 1)
# Initial position of end effector (25, 14.435)
cv2.circle(img, (int(canvasX/2), int(canvasY - 0.5774*(canvasX/2))), radius, (0, 0, 0), -1)
cv2.namedWindow('End Effector Tracker')
cv2. resizeWindow('End Effector Tracker', canvasX+1, canvasY+1)
# Use drawCables mouse callback function, passing time since last
cv2.setMouseCallback('End Effector Tracker', drawCables)
while(1):
    cv2.imshow('End Effector Tracker', img) 
    # print(xCoord, yCoord)
    if cv2.waitKey(20) & 0xFF == 27:
        break
cv2.destroyAllWindows()
