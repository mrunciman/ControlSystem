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
centreX = int(canvasX/2)
centreY = int(canvasY - 0.5774*(canvasX/2))
# Create background image
bkGd = np.zeros(( canvasY+1, canvasX+1, 3), np.uint8)
bkGd[:,:] = (255, 255, 255)
windowName = 'End Effector Tracker'
xCoord = 25
yCoord = 14.435
stopFlag = False
timeDiff = 0.01 # TEMPORARY FIX
# Vertices of equilateral:
vt1 = [0, canvasY]
vt2 = [canvasX, canvasY]
vt3 = [int(canvasX/2), 0]
vts = np.array([vt1, vt2, vt3])
path = mpltPath.Path(vts)
vts = vts.reshape((-1,1,2))
# A flag for when mouse is up or down
mouseDown = False
# Radius of circles that are drawn 
radius = 3
# neighbourhood = (x-2, y-2), (x+2, y-2), (x+2, y+2), (x-2, y+2)
# Check if position is within neighbourhood on down click
# If inside, move end effector and log.

# Mouse callback function
def drawCables(event, x, y, flags, param):
    global prevMillis
    global timeDiff
    global mouseDown
    global xCoord
    global yCoord
    now = time.time()
    # Save time since beginning code in ms
    numMillis = now - start
    numMillis = numMillis*1000 # rounding error?
    timeDiff = numMillis - prevMillis
    prevMillis = numMillis
    # Check if mouse button was pressed down (event 1)
    if event == 1:
        mouseDown = True
        bkGd[:,:] = (255, 255, 255)
        cv2.polylines(bkGd, [vts], True, (0, 0, 0), 1)
        cv2.circle(bkGd, (x, y), radius, (0, 0, 0), -1)
        cv2.line(bkGd, (vt1[0], vt1[1]), (x, y), (0, 128, 0), 1)
        cv2.line(bkGd, (vt2[0], vt2[1]), (x, y), (0, 128, 0), 1)
        cv2.line(bkGd, (vt3[0], vt3[1]), (x, y), (0, 128, 0), 1)
        # print("Inside, Down")
    # Check if mouse button released (event 4) and redraw
    if event == 4:
        mouseDown = False
        cv2.line(bkGd, (vt1[0], vt1[1]), (x, y), (0, 0, 200), 1)
        cv2.line(bkGd, (vt2[0], vt2[1]), (x, y), (0, 0, 200), 1)
        cv2.line(bkGd, (vt3[0], vt3[1]), (x, y), (0, 0, 200), 1)
        # print("Inside, Up")
    # Check if point is inside triangle workspace
    inside = path.contains_point([x, y])
    if inside == True:
        if mouseDown == True:
            # print("Dooooon")
            if event == 0:
                cv2.circle(bkGd, (x, y), radius, (0, 0, 0), -1)
                # Draw cables 
                # bkGd[:,:] = (255, 255, 255)
                # cv2.polylines(bkGd, [vts], True, (0, 0, 0), 1)
                # cv2.line(bkGd, (vt1[0], vt1[1]), (x, y), (0, 128, 0), 1)
                # cv2.line(bkGd, (vt2[0], vt2[1]), (x, y), (0, 128, 0), 1)
                # cv2.line(bkGd, (vt3[0], vt3[1]), (x, y), (0, 128, 0), 1)
                y = canvasY - y
                xCoord = x*resolution
                yCoord = y*resolution
                # print(xCoord, yCoord)
                with open(fileName, 'a', newline='') as posLog:
                    logger = csv.writer(posLog)
                    logger.writerow([event, x, y, now, timeDiff])




# Lines below will be in controlSytem loop
# Call function to instantiate canvas and set callback function
def createTracker():
    # Create a blank image, a window and bind the function to window
    cv2.polylines(bkGd, [vts], True, (0, 0, 0), 1)
    # Initial position of end effector (25, 14.435)
    cv2.circle(bkGd, (centreX, centreY), radius, (0, 0, 0), -1)
    cv2.line(bkGd, (vt1[0], vt1[1]), (centreX, centreY), (0, 128, 0), 1)
    cv2.line(bkGd, (vt2[0], vt2[1]), (centreX, centreY), (0, 128, 0), 1)
    cv2.line(bkGd, (vt3[0], vt3[1]), (centreX, centreY), (0, 128, 0), 1)
    # Create a window with  given name
    cv2.namedWindow(windowName)
    cv2. resizeWindow(windowName, canvasX+1, canvasY+1)# CAN THIS BE REMOVED?
    # Bind drawCables mouse callback function to window
    cv2.setMouseCallback(windowName, drawCables)
    

def iterateTracker():
    cv2.imshow(windowName, bkGd)
    # print(xCoord, yCoord)
    stopFlag = False
    if cv2.waitKey(20) & 0xFF == 27:
        stopFlag = True
        cv2.destroyAllWindows()
    return xCoord, yCoord, timeDiff, stopFlag

# nameWindow = 'End Effector Tracker'
# createTracker(nameWindow)
# while(1):
#     cv2.imshow(nameWindow, bkGd) 
    
#     print(xCoord, yCoord)
#     if cv2.waitKey(20) & 0xFF == 27:
#         break
# cv2.destroyAllWindows()

