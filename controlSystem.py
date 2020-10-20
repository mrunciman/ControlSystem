"""
Read in new position from master
Calculate cable lengths at target.
Calcuate volumes at target.
Calculate desired velocity to reach position in given time
    Use Time equal to reciprocal of master sampling frequency
Calculate rate of cable length change
    Use J and pseudoinverse of J
Calculate rate of volume change (volume flow rate)
Calculate speed of each pump piston
Set step frequency of individual pumps to alter speed
"""

from arduinoInterface import ardInterfacer
from kinematics import kineSolver
from mouseGUI import mouseTracker
from ardLogger import ardLogger
from streaming import optiTracker
import csv
# import math
# import random

############################################################
# Instantiate classes:
sideLength = 18.911 # mm, from workspace2 model

kineSolve = kineSolver(sideLength)
mouseTrack = mouseTracker(sideLength)
ardLogging = ardLogger()
opTrack = optiTracker()

############################################################
pathCounter = 0
cycleCounter = 0
with open ("C:/Users/mruncima/OneDrive - Imperial College London/Imperial/Fluidic Control/ControlSystem/paths/circCoords 2020-10-19.csv",\
    mode = 'r', newline='') as pathFile:
    coordList = csv.reader(pathFile, delimiter=',')
    xCoordList = []
    yCoordList = []
    for row in coordList:
        xCoord = float(row[0])
        yCoord = float(row[1])
        xCoordList.append(xCoord)
        yCoordList.append(yCoord)
        
# print(xCoordList[0], yCoordList[0])

mouseTrack.xCoord = xCoordList[0]
mouseTrack.yCoord = yCoordList[0]
mouseTrack.xPathCoords = xCoordList
mouseTrack.yPathCoords = yCoordList



# Initialise variables 
SAMP_FREQ = 1/kineSolve.timeStep

flagStop = False

# Target must be cast as immutable type (float, in this case) so that 
# the current position doesn't update at same time as target
currentX = mouseTrack.xCoord
currentY = mouseTrack.yCoord
targetX = mouseTrack.xCoord
targetXTest = targetX
targetY = mouseTrack.yCoord
toggleDirection = 1
delayCount = 0
delayLim = 100
# inLim1 = 0.08 # 32% of side length (17 mm contraction)
# inLim2 = 0.92 # 92% of side length (2 mm contraction)
oscStep = 0.1
maxContract = 17
minContract = 2
leftLim = kineSolve.sideLength - maxContract # 17 mm contraction wrt LHS vertex
rightLim = kineSolve.sideLength - minContract # 2 mm contraction wrt LHS vertex
initialXFlag = False
randoPosition = False
# Count number of reps 
halfCycles = 0
noCycles = 30

# Initialise cable length variables at home position
cVolL, cVolR, cVolT = 0, 0, 0
cableL, cableR, cableT = kineSolve.sideLength, kineSolve.sideLength, kineSolve.sideLength
[targetL, targetR, targetT, cJaco, cJpinv] = kineSolve.cableLengths(currentX, currentY, targetX, targetY)
repJaco = cJaco
repJpinv = cJpinv

# Set current volume (ignore tSpeed and step values) 
[cVolL, tSpeedL, tStepL, LcRealL, angleL] = kineSolve.length2Vol(cableL, targetL)
[cVolR, tSpeedR, tStepR, LcRealR, angleR] = kineSolve.length2Vol(cableR, targetR)
[cVolT, tSpeedT, tStepT, LcRealT, angleT] = kineSolve.length2Vol(cableT, targetT)

[tVolL, vDotL, dDotL, fStepL, tStepL, tSpeedL, LcRealL, angleL] = kineSolve.volRate(cVolL, cableL, targetL)
[tVolR, vDotR, dDotR, fStepR, tStepR, tSpeedR, LcRealR, angleR] = kineSolve.volRate(cVolR, cableR, targetR)
[tVolT, vDotT, dDotT, fStepT, tStepT, tSpeedT, LcRealT, angleT] = kineSolve.volRate(cVolT, cableT, targetT)
[LStep, RStep, TStep] = kineSolve.freqScale(fStepL, fStepR, fStepT)
LStep, RStep, TStep = 0, 0, 0

# Set initial pressure and calibration variables
pressL, pressR, pressT = 0, 0, 0
cTimeL, cTimeR, cTimeT = 0, 0, 0
timeL, timeR, timeT = 0, 0, 0

# Current position
cStepL = tStepL
cStepR = tStepR
cStepT = tStepT
realStepL, realStepR, realStepT = 0, 0, 0
angleL, angleR, angleT = 0, 0, 0
cRealStepL = realStepL
cRealStepR = realStepR
cRealStepT = realStepT
dStepL, dStepR, dStepT  = 0, 0, 0

StepNoL, StepNoR, StepNoT = tStepL, tStepR, tStepT

###############################################################
# Connect to Arduinos

# Set COM port for each pump
lhsCOM = 19
rhsCOM = 18
topCOM = 17
closeMessage = "Closed"
try:
    ardIntLHS = ardInterfacer("LHS", lhsCOM)
    reply = ardIntLHS.connect()
    print(reply)
    ardIntRHS = ardInterfacer("RHS", rhsCOM)
    reply = ardIntRHS.connect()
    print(reply)
    ardIntTOP = ardInterfacer("TOP", topCOM)
    reply = ardIntTOP.connect()
    print(reply)

    #############################################################
    # Calibrate arduinos for zero volume - maintain negative pressure for 4 seconds
    calibL = False
    calibR = False
    calibT = False
    calibration = False
    # Calibration ON if TRUE below:
    while (calibration != False):
        if not(calibL):
            [realStepL, pressL, timeL] = ardIntLHS.listenZero(calibL)
        if not(calibR):
            [realStepR, pressR, timeR] = ardIntRHS.listenZero(calibR)
        if not(calibT):
            [realStepT, pressT, timeT] = ardIntTOP.listenZero(calibT)
        print(realStepL, pressL)
        print(realStepR, pressR)
        print(realStepT, pressT)
        if (realStepL == "0000LHS"):
            calibL = True
        if (realStepR == "0000RHS"):
            calibR = True
        if (realStepT == "0000TOP"):
            calibT = True
        if (calibL * calibR * calibT == 1):
            calibration = True
            # Send 0s instead of StepNo as signal that calibration done
            ardLogging.ardLog(realStepL, LcRealL, angleL, 0, pressL, timeL,\
                realStepR, LcRealR, angleR, 0, pressR, timeR,\
                realStepT, LcRealT, angleT, 0, pressT, timeT)
        else:
            ardLogging.ardLog(realStepL, LcRealL, angleL, StepNoL, pressL, timeL,\
                realStepR, LcRealR, angleR, StepNoR, pressR, timeR,\
                realStepT, LcRealT, angleT, StepNoT, pressT, timeT)


    ################################################################
    # Begin main loop
    try:
        # Bring up GUI
        mouseTrack.createTracker()
        while(flagStop == False):
            XYPathCoords = [xCoordList[pathCounter], yCoordList[pathCounter]]
            pathCounter += 1
            if pathCounter >= len(xCoordList):
                pathCounter = 0
                cycleCounter += 1
                if cycleCounter > 3:
                    break

            [targetX, targetY, tMillis, flagStop] = mouseTrack.iterateTracker(pressL, pressR, pressT, XYPathCoords)
            targetX = XYPathCoords[0]
            targetY = XYPathCoords[1]
            print(targetX, targetY)
            tSecs = tMillis/1000

            # Do cable and syringe calculations:
            # Get target lengths and Jacobian from target point

            ##########################################
            # Manually set targets here

            ###
            # Discretise input:
            # targetY = 0
            # targetX = kineSolve.sideLength*round(targetX/(kineSolve.sideLength/10))/10
            ###

            ###
            ## Set random target
            # if initialXFlag == True:
            #     if randoPosition == False:
            #         targetXTest = random.uniform(leftLim, rightLim)
            #         randoPosition = True
            #     else:
            #         targetXTest = currentX
            #         if delayCount < delayLim/2:
            #             delayCount += 1
            #         else:
            #             delayCount = 0
            #             randoPosition = False
            ###

            ###
            ## Initially pause at point determined by GUI
            # if initialXFlag == False:
            #     targetXTest = mouseTrack.xCoord
            #     if delayCount < delayLim:
            #         delayCount += 1
            #     else:
            #         delayCount = 0
            #         initialXFlag = True
            ###

            ## Ensure 1 decimal place
            # targetXTest = round(100*targetXTest)/100 # Will only allow a step oscStep with 2 decimal places
            # targetX = targetXTest

            # Limit input
            # if targetX <= minContract:
            #     targetX = minContract
            # elif targetX >=maxContract:
            #     targetX = maxContract
            # Add proximity restriction to top vertex as well, where both x = side/2 and y = maxY

            # Return target cable lengths at target coords and jacobian at current coords
            [targetL, targetR, targetT, cJaco, cJpinv] = kineSolve.cableLengths(currentX, currentY, targetX, targetY)
            # Get cable speeds using Jacobian at current point and calculation of input speed
            [lhsV, rhsV, topV, actualX, actualY] = kineSolve.cableSpeeds(currentX, currentY, targetX, targetY, cJaco, cJpinv, tSecs)
            # Compare to naive/no speed control calculation of before:
            # [tVolL1, vDotL1, dDotL1, fStepL1, tStepL1, tSpeedL1, LcRealL1, angleL1] = kineSolve.volRate(cVolL, cableL, targetL)
            # [tVolR1, vDotR1, dDotR1, fStepR1, tStepR1, tSpeedR1, LcRealR1, angleR1] = kineSolve.volRate(cVolR, cableR, targetR)
            # [tVolT1, vDotT1, dDotT1, fStepT1, tStepT1, tSpeedT1, LcRealT1, angleT1] = kineSolve.volRate(cVolT, cableT, targetT)
            # Find actual target cable lengths based on scaled cable speeds that result in 'actual' coords
            [scaleTargL, scaleTargR, scaleTargT, repJaco, repJpinv] = kineSolve.cableLengths(currentX, currentY, actualX, actualY)
            # Get volumes, volrates, syringe speeds, pulse freq & step counts estimate for each pump
            [tVolL, vDotL, dDotL, fStepL, tStepL, tSpeedL, LcRealL, angleL] = kineSolve.volRate(cVolL, cableL, scaleTargL)
            [tVolR, vDotR, dDotR, fStepR, tStepR, tSpeedR, LcRealR, angleR] = kineSolve.volRate(cVolR, cableR, scaleTargR)
            [tVolT, vDotT, dDotT, fStepT, tStepT, tSpeedT, LcRealT, angleT] = kineSolve.volRate(cVolT, cableT, scaleTargT)
            # print("Normal", tStepL, tStepR, tStepT)

            # CALCULATE FREQS FROM VALID STEP NUMBER
            # tStepL is target pump position, cStepL is current, speed controlled position.
            fStepL = (tStepL - cStepL)*SAMP_FREQ
            fStepR = (tStepR - cStepR)*SAMP_FREQ
            fStepT = (tStepT - cStepT)*SAMP_FREQ
            [LStep, RStep, TStep] = kineSolve.freqScale(fStepL, fStepR, fStepT)
            StepNoL += LStep
            StepNoR += RStep # RStep = dStepR scaled for speed (w rounding differences)
            StepNoT += TStep
            # Send scaled step number to arduinos:
            ardIntLHS.sendStep(StepNoL)
            ardIntRHS.sendStep(StepNoR)
            ardIntTOP.sendStep(StepNoT)

            # Update current position, cable lengths, and volumes as previous targets
            currentX = actualX
            currentY = actualY
            cableL = scaleTargL
            cableR = scaleTargR
            cableT = scaleTargT
            cVolL = tVolL
            cVolR = tVolR
            cVolT = tVolT
            cStepL = StepNoL
            cStepR = StepNoR
            cStepT = StepNoT
            ardLogging.ardLog(realStepL, LcRealL, angleL, StepNoL, pressL, timeL,\
                realStepR, LcRealR, angleR, StepNoR, pressR, timeR,\
                realStepT, LcRealT, angleT, StepNoT, pressT, timeT)



            [realStepL, pressL, timeL] = ardIntLHS.listenReply()
            [realStepR, pressR, timeR] = ardIntRHS.listenReply()
            [realStepT, pressT, timeT] = ardIntTOP.listenReply()
            # print("Targ Pos: ", StepNoL, StepNoR, StepNoT)
            # print("Scal Pos: ", StepNoL, StepNoR, StepNoT)
            # print("Real Pos: ", realStepL, realStepR, realStepT)
            # diffStepL = int(realStepL) - cRealStepL
            # diffStepR = int(realStepR)- cRealStepR
            # diffStepT = int(realStepT) - cRealStepT
            # Check if muscles stop at same time - suggests IK working
            # print(diffStepL==0, diffStepR==0, diffStepT==0)
            # print(diffStepL, diffStepR, diffStepT)
            # loopTimeL = int(timeL) - cTimeL
            # loopTimeR = int(timeR) - cTimeR
            # loopTimeT = int(timeT) - cTimeT
            # cRealStepL = int(realStepL)
            # cRealStepR = int(realStepR)
            # cRealStepT = int(realStepT)
            # cTimeL = int(timeL)
            # cTimeR = int(timeR)
            # cTimeT = int(timeT)
            # print("Pressure: ", pressL, pressR, pressT)
            # print("Real Pos: ", realStepL, realStepR, realStepT)
            # print(targetL, targetR, targetT)
            # print(tVolL, tVolR, tVolT)

        flagStop = mouseTrack.closeTracker()

    except KeyboardInterrupt:
        pass
    
except KeyboardInterrupt:
    pass

finally:
    ###########################################################################
    # Stop program
    # Disable pumps and set them to idle state
    try:
        # Save values gathered from arduinos
        ardLogging.ardLog(realStepL, LcRealL, angleL, StepNoL, pressL, timeL,\
            realStepR, LcRealR, angleR, StepNoR, pressR, timeR,\
            realStepT, LcRealT, angleT, StepNoT, pressT, timeT)
        ardLogging.ardSave()

        if ardIntLHS.ser.is_open:
            ardIntLHS.sendStep(closeMessage)
            [realStepL, pressL, timeL] = ardIntLHS.listenReply()
            print(realStepL, pressL, timeL)

        if ardIntRHS.ser.is_open:
            ardIntRHS.sendStep(closeMessage)
            [realStepR, pressR, timeR] = ardIntRHS.listenReply()
            print(realStepR, pressR, timeR)
        
        if ardIntTOP.ser.is_open:
            ardIntTOP.sendStep(closeMessage)
            [realStepT, pressT, timeT] = ardIntTOP.listenReply()
            print(realStepT, pressT, timeT)

    except NameError:
        reply = ardIntLHS.connect()
        print(reply)
        reply = ardIntRHS.connect()
        print(reply)
        reply = ardIntTOP.connect()
        print(reply)

    # Close serial connections
    ardIntLHS.ser.close()
    ardIntRHS.ser.close()
    ardIntTOP.ser.close()

