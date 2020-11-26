
import csv
import numpy as np
import math as mt

sideLength = 18.91129205
circRad = 2.5
circCentX = sideLength/2
circCentY = (sideLength/2)*mt.tan(mt.pi/6)

noSteps = 360*2
rotStep = np.linspace(0, 2*mt.pi*(1 - 1/noSteps), noSteps)
print(len(rotStep))
xPath = circRad*np.cos(rotStep) + circCentX
yPath = circRad*np.sin(rotStep) + circCentY
# circCoords = [x, y]
# writematrix(circCoords,'circCoords.csv')

with open("C:/Users/mruncima/OneDrive - Imperial College London/Imperial/Fluidic Control/ControlSystem/paths/circCoords 2020-10-20.csv",\
    mode ='w', newline='') as pathGenerator: 
    pathGen = csv.writer(pathGenerator)
    for j in range(len(xPath)):
        pathGen.writerow([xPath[j], yPath[j]])

pathCounter = 0
cycleCounter = 0

with open ("C:/Users/mruncima/OneDrive - Imperial College London/Imperial/Fluidic Control/ControlSystem/paths/circCoords 2020-10-20.csv",\
    mode = 'r', newline='') as pathFile:
    coordList = csv.reader(pathFile, delimiter=',')
    xCoordList = []
    yCoordList = []
    for row in coordList:
        xCoord = row[0]
        yCoord = row[1]
        xCoordList.append(xCoord)
        yCoordList.append(yCoord)
        # Multiply by mouseGUI resolution to get pixel values

# while cycleCounter < 3:
    # print(xCoordList[pathCounter], yCoordList[pathCounter])
    # pathCounter += 1
    # if pathCounter >= len(xCoordList):
    #     pathCounter = 0
    #     cycleCounter += 1
        # print(cycleCounter)



