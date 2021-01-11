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
import traceback
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

# Count number of reps 
halfCycles = 0
noCycles = 30
# Use different methods for different paths

xPath = []
yPath = []
# Read directly from file for speed?
with open('paths/circPath 2021-01-06 11-44-05 0.625mmRad30Reps.csv', newline = '') as csvPath:
    coordReader = csv.reader(csvPath)
    for row in coordReader:
        xPath.append(float(row[0]))
        yPath.append(float(row[1]))


# Use mouse as primary?
useMouse = False

if not useMouse:
    mouseTrack.xCoord = xPath[0]
    mouseTrack.yCoord = yPath[0]
    mouseTrack.xPathCoords = xPath[0: int(len(xPath)/noCycles)]  #Down-sample path here for display
    mouseTrack.yPathCoords = yPath[0: int(len(yPath)/noCycles)]

# Initialise variables 
SAMP_FREQ = 1/kineSolve.timeStep
flagStop = False

# Target must be cast as immutable type (float, in this case) so that 
# the current position doesn't update at same time as target
currentX = mouseTrack.xCoord
currentY = mouseTrack.yCoord
targetX = mouseTrack.xCoord
targetY = mouseTrack.yCoord
toggleDirection = 1
delayCount = 0
delayLim = 200

initialXFlag = False
randoPosition = False


# Initialise cable length variables at home position
cVolL, cVolR, cVolT = 0, 0, 0
cableL, cableR, cableT = kineSolve.sideLength, kineSolve.sideLength, kineSolve.sideLength
[targetL, targetR, targetT, cJaco, cJpinv] = kineSolve.cableLengths(currentX, currentY, targetX, targetY)
print(targetL, targetR, targetT)
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
lhsCOM = 8
rhsCOM = 6
topCOM = 7
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
    # Has the mechanism been calibrated/want to run without calibration?:
    calibrated = True
    # Perform calibration:
    while (not calibrated):
        # if not(calibL):
        [realStepL, pressL, timeL] = ardIntLHS.listenZero(calibL, pressL, timeL)
        # if not(calibR):
        [realStepR, pressR, timeR] = ardIntRHS.listenZero(calibR, pressR, timeR)
        # if not(calibT):
        [realStepT, pressT, timeT] = ardIntTOP.listenZero(calibT, pressT, timeT)
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
            calibrated = True
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

    # Bring up GUI
    mouseTrack.createTracker()
    while(flagStop == False):

        if delayCount < delayLim:
            delayCount += 1
            pathCounter = 0
        XYPathCoords = [xPath[pathCounter], yPath[pathCounter]]
        pathCounter += 1

        # Change this part - no need to go back to start of path, path will contain all reps
        if pathCounter >= len(xPath):
            # pathCounter = 0
            # print(cycleCounter)
            # cycleCounter += 1
            # if cycleCounter > noCycles:
            break

        if useMouse:
            XYPathCoords = None

        [targetX, targetY, tMillis, flagStop] = mouseTrack.iterateTracker(pressL, pressR, pressT, XYPathCoords)
        tSecs = tMillis/1000
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



except TypeError as exTE:
    tb_linesTE = traceback.format_exception(exTE.__class__, exTE, exTE.__traceback__)
    tb_textTE = ''.join(tb_linesTE)
    print(tb_textTE)

except KeyboardInterrupt as ctrlC:
    print(ctrlC)

    # except Exception as ex:
    #     tb_lines = traceback.format_exception(ex.__class__, ex, ex.__traceback__)
    #     tb_text = ''.join(tb_lines)
    #     print(tb_text)
    

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
        flagStop = mouseTrack.closeTracker()

        if ardIntLHS.ser.is_open:
            ardIntLHS.sendStep(closeMessage)

        if ardIntRHS.ser.is_open:
            ardIntRHS.sendStep(closeMessage)
        
        if ardIntTOP.ser.is_open:
            ardIntTOP.sendStep(closeMessage)

        [realStepL, pressL, timeL] = ardIntLHS.listenReply()
        print(realStepL, pressL, timeL)
        [realStepR, pressR, timeR] = ardIntRHS.listenReply()
        print(realStepR, pressR, timeR)
        [realStepT, pressT, timeT] = ardIntTOP.listenReply()
        print(realStepT, pressT, timeT)

    except NameError:
        reply = ardIntLHS.connect()
        print(reply)
        reply = ardIntRHS.connect()
        print(reply)
        reply = ardIntTOP.connect()
        print(reply)

    except TypeError as exTE:
        tb_linesTE = traceback.format_exception(exTE.__class__, exTE, exTE.__traceback__)
        tb_textTE = ''.join(tb_linesTE)
        # print(tb_textTE)

    # Close serial connections
    ardIntLHS.ser.close()
    ardIntRHS.ser.close()
    ardIntTOP.ser.close()

