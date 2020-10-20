
import csv

pathCounter = 0
cycleCounter = 0


with open ("C:/Users/msrun/OneDrive - Imperial College London/Imperial/Fluidic Control/ControlSystem/paths/circCoords 2020-10-19.csv",\
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

while cycleCounter < 3:
    print(xCoordList[pathCounter], yCoordList[pathCounter])
    pathCounter += 1
    if pathCounter >= len(xCoordList):
        pathCounter = 0
        cycleCounter += 1
        print(cycleCounter)



