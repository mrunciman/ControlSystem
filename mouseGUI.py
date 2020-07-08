import cv2
import numpy as np
import csv
import time
import matplotlib.path as mpltPath



# Create a file in 'log' directory and empty contents (if it already exists)
# Append timestamp in ms to name
location = "C:/Users/msrun/OneDrive - Imperial College London/Imperial/Fluidic Control/ControlSystem/logs"
# logTime = time.time()
logTime = time.strftime("%Y-%m-%d-_-%H-%M-%S")
fileName = location + "/positions " + logTime + ".csv" # USE THIS IN REAL TESTS
# fileName = 'test.csv' # For test purposes
with open(fileName, mode ='w', newline='') as posLog1: 
    logger1 = csv.writer(posLog1)
    logger1.writerow(['Event', 'X', 'Y', 'Timestamp', 'ms Since Last'])


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
# xCoord = 25
# yCoord = 14.435

# start = time.time()
# prevMillis = 0
# stopFlag = False
# timeDiff = 0.01 # TEMPORARY FIX INITIAL VALUE


# Vertices of equilateral:
vt1 = [0, canvasY]
vt2 = [canvasX, canvasY]
vt3 = [int(canvasX/2), 0]
vts = np.array([vt1, vt2, vt3])
path = mpltPath.Path(vts)
vts = vts.reshape((-1,1,2))
# A flag for when mouse is up or down
# mouseDown = False
# Radius of circles that are drawn 
radius = 3
# Create an 2n+1 pixel side square around current point
n = 10

font = cv2.FONT_HERSHEY_SIMPLEX
fontscale = 0.5
colour = (0, 128, 0)
thick = 1

class mouseTracker:

    def __init__(self):
        # Give initial values for when class instance is made
        # Cast coordinates as floats for immutability, which allows tracking
        self.xCoord = float(centreX*resolution)
        self.yCoord = float(0.5774*centreX*resolution)
        self.xPix = centreX
        self.yPix = centreY
        self.mouseDown = False
        self.touchDown = False
        self.start = time.time()
        self.timeDiff = 0
        self.prevMillis = 0
        self.stopFlag = False
        self.logData = []
        self.flag = 0
        self.param = 0

    # Mouse callback function
    def drawCables(self, event, x, y, flags, param):
        self.flag = flags
        self.param = param
        now = time.time()
        # Save time since beginning code in ms
        numMillis = now - self.start #Still in seconds
        numMillis = numMillis*1000
        self.timeDiff = numMillis - self.prevMillis
        self.prevMillis = numMillis

        # Draw a 5pixel side square around current point
        p1 = [self.xPix-n, self.yPix-n]
        p2 = [self.xPix+n, self.yPix-n]
        p3 = [self.xPix+n, self.yPix+n]
        p4 = [self.xPix-n, self.yPix+n]
        neighbour = np.array([p1, p2, p3, p4])
        neighPath = mpltPath.Path(neighbour)
        neighShape = neighbour.reshape((-1,1,2))
        cv2.polylines(bkGd, [neighShape], True, (0, 0, 0), 1)
        # cv2.polylines(bkGd, [neighShape], True, (0, 0, 0), 1)
        # Check if position is within neighbourhood on down click
        # If inside, move end effector and log.
        touching = neighPath.contains_point([x, y])
        # Check if point is inside triangle workspace
        inside = path.contains_point([x, y])
        # Check if mouse button was pressed down (event 1)
        if event == 1:
            self.mouseDown = True
            if touching == True:
                self.touchDown = True

        # Check if mouse button released (event 4) and redraw
        if event == 4:
            self.mouseDown = False
            self.touchDown = False
            # If released inside triangle and within
            # end effector handle, redraw cables red
            if inside == True:
                if touching == True:
                    cv2.line(bkGd, (vt1[0], vt1[1]), (x, y), (0, 0, 200), 1)
                    cv2.line(bkGd, (vt2[0], vt2[1]), (x, y), (0, 0, 200), 1)
                    cv2.line(bkGd, (vt3[0], vt3[1]), (x, y), (0, 0, 200), 1)

        if inside == True:
            if self.mouseDown == True:
                if self.touchDown == True:
                    # Check if moving
                    if event == 0:
                        cv2.circle(bkGd, (x, y), radius, (0, 0, 0), -1)
                        # Draw cables 
                        bkGd[:,:] = (255, 255, 255)
                        cv2.circle(bkGd, (x, y), radius, (0, 0, 0), -1)
                        cv2.polylines(bkGd, [vts], True, (0, 0, 0), 1)
                        cv2.line(bkGd, (vt1[0], vt1[1]), (x, y), (0, 128, 0), 1)
                        cv2.line(bkGd, (vt2[0], vt2[1]), (x, y), (0, 128, 0), 1)
                        cv2.line(bkGd, (vt3[0], vt3[1]), (x, y), (0, 128, 0), 1)
                        yPrime = canvasY - y
                        self.xPix = x
                        self.xCoord = x*resolution
                        # self.Coords[0] = x*resolution
                        self.yPix = y
                        self.yCoord = yPrime*resolution
                        # self.Coords[1] = yPrime*resolution
                        # Collect tdata in list to be exported on exit
                        self.logData.append([event] + [self.xCoord] + [self.yCoord] + [now] + [self.timeDiff])
        else:
            self.touchDown = False




# Lines below will be in controlSytem loop
# Call function to instantiate canvas and set callback function
    def createTracker(self):
        # Create a blank image, a window and bind the function to window
        cv2.polylines(bkGd, [vts], True, (0, 0, 0), 1)
        # Initial position of end effector (25, 14.435)
        cv2.circle(bkGd, (centreX, centreY), radius, (0, 0, 0), -1)
        cv2.line(bkGd, (vt1[0], vt1[1]), (centreX, centreY), (0, 128, 0), 1)
        cv2.line(bkGd, (vt2[0], vt2[1]), (centreX, centreY), (0, 128, 0), 1)
        cv2.line(bkGd, (vt3[0], vt3[1]), (centreX, centreY), (0, 128, 0), 1)
        # Create a window with  given name
        cv2.namedWindow(windowName)
        # Bind drawCables mouse callback function to window
        cv2.setMouseCallback(windowName, self.drawCables)

    def iterateTracker(self):
        # Bind drawCables mouse callback function to window
        cv2.setMouseCallback(windowName, self.drawCables)
        posText = "({:.2f}, {:.2f})".format(self.xCoord, self.yCoord)
        placement = (self.xPix-60, self.yPix-15)
        cv2.putText(bkGd, posText, placement, font, fontscale, colour, thick, cv2.LINE_AA)
        cv2.imshow(windowName, bkGd)
        if cv2.waitKey(20) & 0xFF == 27:
            self.stopFlag = True
            # cv2.destroyAllWindows()
            # with open(fileName, 'a', newline='') as posLog:
            #     logger = csv.writer(posLog)
            #     logger.writerows(self.logData)
        return self.xCoord, self.yCoord, self.timeDiff, self.stopFlag

    def closeTracker(self):
        # cv2.waitKey = 27
        self.stopFlag = True
        cv2.destroyAllWindows()
        with open(fileName, 'a', newline='') as posLog:
            logger = csv.writer(posLog)
            logger.writerows(self.logData)
        return self.stopFlag


