import cv2
import numpy as np
import csv
import time
from kinematics import kine
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


font = cv2.FONT_HERSHEY_SIMPLEX
fontscale = 0.5
colour = (0, 128, 0)
thick = 1

class mouseTracker(kine):

    def __init__(self):
        # Resolution in mm/pixel (Geomagic touch res ~0.055 mm)
        self.resolution = 0.03
        self.sideLength = kine.sideLength0
        self.canvasX = int(self.sideLength/self.resolution)
        self.canvasY = int(1.7321*(self.canvasX/2))
        self.centreX = int(self.canvasX/2)
        self.centreY = int(self.canvasY - 0.5774*(self.canvasX/2))
        # Create background image
        self.bkGd = np.zeros(( self.canvasY+1, self.canvasX+1, 3), np.uint8)
        self.bkGd[:,:] = (255, 255, 255)
        self.windowName = 'End Effector Tracker'
        # Vertices of equilateral:
        self.vt1 = [0, self.canvasY]
        self.vt2 = [self.canvasX, self.canvasY]
        self.vt3 = [int(self.canvasX/2), 0]
        self.vts = np.array([self.vt1, self.vt2, self.vt3])
        self.path = mpltPath.Path(self.vts)
        self.vts = self.vts.reshape((-1,1,2))
        # Radius of circles that are drawn 
        self.radius = 3
        # Create an 2num+1 pixel side square around current point
        self.num = 10



        # Give initial values for when class instance is made
        # Cast coordinates as floats for immutability, which allows tracking
        self.xCoord = self.sideLength*0.9#sideLength*0.9#float(centreX*resolution)
        self.yCoord = 1#float(0.5774*centreX*resolution)#
        self.xPix = int(self.xCoord/self.resolution)#centreX#750#int(self.xCoord/resolution)#
        self.yPix = self.canvasY - int(self.yCoord/self.resolution)#centreY#canvasY-5#
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
        p1 = [self.xPix-self.num, self.yPix-self.num]
        p2 = [self.xPix+self.num, self.yPix-self.num]
        p3 = [self.xPix+self.num, self.yPix+self.num]
        p4 = [self.xPix-self.num, self.yPix+self.num]
        neighbour = np.array([p1, p2, p3, p4])
        neighPath = mpltPath.Path(neighbour)
        neighShape = neighbour.reshape((-1,1,2))
        cv2.polylines(self.bkGd, [neighShape], True, (0, 0, 0), 1)
        # cv2.polylines(self.bkGd, [neighShape], True, (0, 0, 0), 1)
        # Check if position is within neighbourhood on down click
        # If inside, move end effector and log.
        touching = neighPath.contains_point([x, y])
        # Check if point is inside triangle workspace
        inside = self.path.contains_point([x, y])
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
                    cv2.line(self.bkGd, (self.vt1[0], self.vt1[1]), (x, y), (0, 0, 200), 1)
                    cv2.line(self.bkGd, (self.vt2[0], self.vt2[1]), (x, y), (0, 0, 200), 1)
                    cv2.line(self.bkGd, (self.vt3[0], self.vt3[1]), (x, y), (0, 0, 200), 1)

        if inside == True:
            if self.mouseDown == True:
                if self.touchDown == True:
                    # Check if moving
                    if event == 0:
                        cv2.circle(self.bkGd, (x, y), self.radius, (0, 0, 0), -1)
                        # Draw cables 
                        self.bkGd[:,:] = (255, 255, 255)
                        cv2.circle(self.bkGd, (x, y), self.radius, (0, 0, 0), -1)
                        cv2.polylines(self.bkGd, [self.vts], True, (0, 0, 0), 1)
                        cv2.line(self.bkGd, (self.vt1[0], self.vt1[1]), (x, y), (0, 128, 0), 1)
                        cv2.line(self.bkGd, (self.vt2[0], self.vt2[1]), (x, y), (0, 128, 0), 1)
                        cv2.line(self.bkGd, (self.vt3[0], self.vt3[1]), (x, y), (0, 128, 0), 1)
                        yPrime = self.canvasY - y
                        self.xPix = x
                        self.xCoord = x*self.resolution
                        # self.Coords[0] = x*resolution
                        self.yPix = y
                        self.yCoord = yPrime*self.resolution
                        # self.Coords[1] = yPrime*resolution
                        # Collect tdata in list to be exported on exit
                        self.logData.append([event] + [self.xCoord] + [self.yCoord] + [now] + [self.timeDiff])
        else:
            self.touchDown = False




# Lines below will be in controlSytem loop
# Call function to instantiate canvas and set callback function
    def createTracker(self):
        # Create a blank image, a window and bind the function to window
        cv2.polylines(self.bkGd, [self.vts], True, (0, 0, 0), 1)
        # Initial position of end effector (25, 14.435)
        cv2.circle(self.bkGd, (self.xPix, self.yPix), self.radius, (0, 0, 0), -1)
        cv2.line(self.bkGd, (self.vt1[0], self.vt1[1]), (self.xPix, self.yPix), (0, 128, 0), 1)
        cv2.line(self.bkGd, (self.vt2[0], self.vt2[1]), (self.xPix, self.yPix), (0, 128, 0), 1)
        cv2.line(self.bkGd, (self.vt3[0], self.vt3[1]), (self.xPix, self.yPix), (0, 128, 0), 1)
        # Create a window with  given name
        cv2.namedWindow(self.windowName)
        # Bind drawCables mouse callback function to window
        cv2.setMouseCallback(self.windowName, self.drawCables)

    def iterateTracker(self):
        # Bind drawCables mouse callback function to window
        cv2.setMouseCallback(self.windowName, self.drawCables)
        posText = "({:.2f}, {:.2f})".format(self.xCoord, self.yCoord)
        placement = (self.xPix-60, self.yPix-15)
        cv2.putText(self.bkGd, posText, placement, font, fontscale, colour, thick, cv2.LINE_AA)
        cv2.imshow(self.windowName, self.bkGd)
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


