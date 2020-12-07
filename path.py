
import csv
import numpy as np
import math as mt
import time
import os


class pathGenerator:
# ALL PATHS MUST START FROM HOME TO BE IN AGREEMENT WITH CALIBRATION ON ARDUINOS
    def __init__(self, triangleSide):

        # Geometry of entry points
        self.sideLength = triangleSide#18.91129205
        self.triHeight = (triangleSide/2)*mt.tan(mt.pi/3)
        self.circCentX = triangleSide/2
        self.circCentY = (triangleSide/2)*mt.tan(mt.pi/6)
        self.xPath = np.array([]) # Poor style but works for now
        self.yPath = np.array([])

        # File name initialisation
        self.location = os.path.dirname(__file__)
        self.logTime = time.strftime("%Y-%m-%d %H-%M-%S")
        self.relative = "paths/genericPath" + self.logTime + str(self.sideLength) + "EqSide.csv"
        self.fileName = []

    def generatePath(self):

        # Prepend path with slow move to initial position 

        # Convert to lists for improved speed when refereneing individual elements
        self.xPath = self.xPath.tolist()
        self.yPath = self.yPath.tolist()

        with open(self.fileName, mode ='w', newline = '') as pathGenny: 
            pathGenr = csv.writer(pathGenny)
            for j in range(len(self.xPath)):
                pathGenr.writerow([self.xPath[j], self.yPath[j]])


    def circlePath(self, numReps):
        noSteps = 360
        circRadius = 2.5
        circRad = circRadius #np.concatenate([np.linspace(0, self.circRadius, self.noSteps), np.linspace(self.circRadius, 0, self.noSteps)])

        self.relative = "paths/circPath " + self.logTime + " " + str(circRadius) + "mmRad" + str(self.sideLength) + "EqSide.csv"
        self.fileName = os.path.join(self.location, self.relative)

        rotStep = np.linspace(0, 2*mt.pi*(1 - 1/(noSteps)), noSteps)
        xPathInter = circRad*np.cos(rotStep) + self.circCentX
        yPathInter = circRad*np.sin(rotStep) + self.circCentY
        self.xPath = np.tile(xPathInter, numReps)
        self.yPath = np.tile(yPathInter, numReps)


    def spiralPath(self, numReps):
        # This is probably wrong - can only do 360 degrees.
        # Better would be Archimedes spiral
        noSteps = 360*2
        circRadius = 1.25
        fwdRadius = np.linspace(0, circRadius, noSteps)
        bwdRadius = np.linspace(circRadius, 0, noSteps)
        spiralRad = np.concatenate((fwdRadius, bwdRadius))

        self.relative = "paths/spiralPath " + self.logTime + " " + str(circRadius) + "mmRad" + str(self.sideLength) + "EqSide.csv"
        self.fileName = os.path.join(self.location, self.relative)

        fwdRot = np.linspace(0, 2*mt.pi*(1 - 1/(noSteps)), noSteps)
        bwdRot = np.linspace(2*mt.pi*(1 - 1/(noSteps)), 0, noSteps)
        rotStep = np.concatenate((fwdRot, bwdRot))

        xPathInter = np.zeros(2*noSteps)
        yPathInter = np.zeros(2*noSteps)
        for i in range(len(spiralRad)):
            xPathInter[i] = spiralRad[i]*np.cos(rotStep[i]) + self.circCentX
            yPathInter[i] = spiralRad[i]*np.sin(rotStep[i]) + self.circCentY
        
        self.xPath = np.tile(xPathInter, numReps)
        self.yPath = np.tile(yPathInter, numReps)


    def rasterScan(self, numReps):
        # Eq triangle in circumcircle, radius determined by longest horizontal line (base)
        scanSpeed = 2 #mm/s
        sideFactor = 0.5 # How long base will be wrt side length
        timeStep = 6/125
        maxLenHorLines = sideFactor*self.sideLength
        # Starting point is on line from centre to corner, sideFactor times this length
        heightFactor = 0.05
        lenVerLines = heightFactor*maxLenHorLines
        # Next line up: starts vertically above last, ends at intersection with triangle

        self.relative = "paths/raster " + self.logTime + " " + str(sideFactor) + "B" + str(heightFactor) + "H"\
            + str(self.sideLength) + "EqSide.csv"
        self.fileName = os.path.join(self.location, self.relative)

        # Initialise variables for loop
        currentX = self.circCentX + 0.5*maxLenHorLines
        currentY = self.circCentY - (maxLenHorLines/2)*mt.tan(mt.pi/6)
        xListInter = np.array([])
        yListInter = np.array([])
        # time per line = dist/speed
        # number of 0.048 s steps = time per line/ time per step (steps per line)
        timePerLine = maxLenHorLines/scanSpeed
        stepsPerLine = round(timePerLine/timeStep)
        horDistPerStep = maxLenHorLines/stepsPerLine
        realHorLen = horDistPerStep*stepsPerLine
        # stopPointX = currentX - horDistPerStep*stepsPerLine

        timePerVerLine = lenVerLines/scanSpeed
        stepsPerVerLine = round(timePerVerLine/timeStep)
        verDistPerStep = lenVerLines/stepsPerLine
        realVerLen = verDistPerStep*stepsPerLine
        # stopPointY = currentY + verDistPerStep*stepsPerLine

        lenReduction = 2*lenVerLines/(mt.tan(mt.pi/3))

        lineLen = maxLenHorLines
        direction = 1 # to toggle direction of travel

        # If length of next horizontal line is less than vertical line height, stop and don't make vertical line.
        while lineLen > lenVerLines:
            startPointX = currentX
            startPointY = currentY
            realHorLen = horDistPerStep*stepsPerLine
            stopPointX = currentX - direction*realHorLen
            stopPointY = currentY + realVerLen

            # Horizontal Line coords
            horLineXArray = np.linspace(startPointX, stopPointX, stepsPerLine)
            horLineYArray = np.array(stepsPerLine*[startPointY]) # same y coord repeated stepsPerLine times
            xListInter = np.append(xListInter, horLineXArray)
            yListInter = np.append(yListInter, horLineYArray)

            # Calculate new line length
            lineLen = lineLen - lenReduction
            timePerLine = lineLen/scanSpeed
            stepsPerLine = round(timePerLine/timeStep)
            horDistPerStep = lineLen/stepsPerLine

            # Vertical Line coords
            if lineLen > lenVerLines:
                verLineXArray = np.array(stepsPerVerLine*[stopPointX]) # same x coord repeated stepsPerVerLine times
                verLineYArray = np.linspace(startPointY, stopPointY, stepsPerVerLine)
                xListInter = np.append(xListInter, verLineXArray)
                yListInter = np.append(yListInter, verLineYArray)
            else:
                break

            direction = -1*direction
            currentX = stopPointX
            currentY = stopPointY

        for i in range(0, numReps):
            # If i is odd, add coordinates in order
            if (i % 2 == 1): 
                self.xPath = np.concatenate((self.xPath, xListInter))
                self.yPath = np.concatenate((self.yPath, yListInter))
                # self.yPath.np.append(yListInter)
            # If i is even, add coords in reverse order
            else:
                self.xPath = np.concatenate((self.xPath, np.flip(xListInter)))
                self.yPath = np.concatenate((self.yPath, np.flip(yListInter)))
                # self.xPath.np.append(np.flip(xListInter))
                # self.yPath.np.append(np.flip(yListInter))
        # self.xPath = np.array(xListInter)
        # self.yPath = np.array(yListInter)
     

    def verticalMoves(self):
        """
        Method to move vertically so that TOP muscle goes from 2 to 17 m contraction with 0.5 mm steps. 
        """
        # self.relative = "paths/vertical " + self.logTime + " 2-17mm 0.5mm step" + str(self.sideLength) + "EqSide.csv"
        # self.fileName = os.path.join(self.location, self.relative)
        currentX = self.sideLength/2
        maxY = (self.sideLength/2)*mt.tan(mt.pi/3)
        currentY = maxY

        minStrain = 2 #mm
        maxStrain = 17 #mm
        stepSize = 0.5 #mm
        strainRange = maxStrain - minStrain
        numSteps = strainRange/stepSize

        for i in range(int(numSteps)):
            # Increment contraction by decreasing Y value
            currentY = currentY - stepSize
            print(currentY, i)






    #####
    # Concentric triangles
    #####
        
    # Start at top of triangle
    # Go round triangle 
    # move vertically to next tip

    # numSubTri = 10 # 10 small triangles inside workspace
    # spacing = (self.sideLength/2)*mt.tan(mt.pi/3)/numSubTri # division of triangle height
    # smallestLength = 0.1*self.sideLength
    # startPointX = self.centreX
    # startPointY = self.centreY + spacing
    # # height to centre from bottom left = (triangleSide/2)*mt.tan(mt.pi/6)
    # biggestLength = 0.9*self.sideLength
    # smallestLength = 0.1*self.sideLength


       # Prepend all paths with move from home to start of path
    


sideLength = 18.911 # mm, from workspace2 model
noCycles = 3
pathGen = pathGenerator(sideLength)
pathGen.verticalMoves()
# pathGen.generatePath()