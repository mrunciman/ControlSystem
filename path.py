
import csv
import numpy as np
import math as mt
import time
import os


class pathGenerator:

    def __init__(self, triangleSide):

        self.sideLength = triangleSide#18.91129205
        self.triHeight = (triangleSide/2)*mt.tan(mt.pi/3)
        self.noSteps = 360*2
        self.circRadius = 1.25
        self.circRad = self.circRadius #np.concatenate([np.linspace(0, self.circRadius, self.noSteps), np.linspace(self.circRadius, 0, self.noSteps)])
        self.circCentX = triangleSide/2
        self.circCentY = (triangleSide/2)*mt.tan(mt.pi/6)

        
        self.location = os.path.dirname(__file__)
        self.logTime = time.strftime("%Y-%m-%d %H-%M-%S")
        self.relative = "paths/circPath " + self.logTime + " " + str(self.circRadius) + "mmRad" + str(self.sideLength) + "EqSide.csv"
        self.fileName = os.path.join(self.location, self.relative)


        self.rotStep = np.linspace(0, 2*mt.pi*(1 - 1/(self.noSteps)), self.noSteps)
        # print(len(rotStep))
        # self.xPath = np.zeros(self.noSteps)
        # self.yPath = np.zeros(self.noSteps)
        # for i in range(int(self.noSteps)):
        self.xPath = self.circRad*np.cos(self.rotStep) + self.circCentX
        self.yPath = self.circRad*np.sin(self.rotStep) + self.circCentY
        # circCoords = [x, y]
        # writematrix(circCoords,'circCoords.csv')

    def generatePath(self):
        with open(self.fileName, mode ='w', newline = '') as pathGenny: 
            pathGen = csv.writer(pathGenny)
            for j in range(len(self.xPath)):
                pathGen.writerow([self.xPath[j], self.yPath[j]])

        # pathCounter = 0
        # cycleCounter = 0

        # with open ("C:/Users/mruncima/OneDrive - Imperial College London/Imperial/Fluidic Control/ControlSystem/paths/circCoords 2020-10-20.csv",\
        #     mode = 'r', newline='') as pathFile:
        #     coordList = csv.reader(pathFile, delimiter=',')
        #     xCoordList = []
        #     yCoordList = []
        #     for row in coordList:
        #         xCoord = row[0]
        #         yCoord = row[1]
        #         xCoordList.append(xCoord)
        #         yCoordList.append(yCoord)
                # Multiply by mouseGUI resolution to get pixel values

        # while cycleCounter < 3:
            # print(xCoordList[pathCounter], yCoordList[pathCounter])
            # pathCounter += 1
            # if pathCounter >= len(xCoordList):
            #     pathCounter = 0
            #     cycleCounter += 1
                # print(cycleCounter)






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


    #####
    # Raster Scan
    ##### 
    def rasterScan(self):
        # Eq triangle in circumcircle, radius determined by longest horizontal line (base)
        scanSpeed = 2 #mm/s
        sideFactor = 0.5 # How long base will be wrt side length
        timeStep = 6/125
        maxLenHorLines = sideFactor*self.sideLength
        # Starting point is on line from centre to corner, sideFactor times this length
        lenVerLines = 0.05*maxLenHorLines
        # Next line up: starts vertically above last, ends at intersection with triangle

        # Initialise variables for loop
        currentX = self.circCentX + 0.5*maxLenHorLines
        currentY = self.circCentY - (maxLenHorLines/2)*mt.tan(mt.pi/6)
        xList = np.array([])
        yList = np.array([])
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
            xList = np.append(xList, horLineXArray)
            yList = np.append(yList, horLineYArray)

            # Calculate new line length
            lineLen = lineLen - lenReduction
            timePerLine = lineLen/scanSpeed
            stepsPerLine = round(timePerLine/timeStep)
            horDistPerStep = lineLen/stepsPerLine

            # Vertical Line coords
            if lineLen > lenVerLines:
                verLineXArray = np.array(stepsPerVerLine*[stopPointX]) # same x coord repeated stepsPerVerLine times
                verLineYArray = np.linspace(startPointY, stopPointY, stepsPerVerLine)
                xList = np.append(xList, verLineXArray)
                yList = np.append(yList, verLineYArray)
            else:
                break

            direction = -1*direction
            currentX = stopPointX
            currentY = stopPointY

        self.xPath = np.array(xList)
        self.yPath = np.array(yList)
    
