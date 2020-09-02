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

from arduinoInterface import connect, sendStep, listenZero, listenReply
from kinematics import kine #cableLengths, volRate, freqScale, length2Vol
from mouseGUI import mouseTracker
from ardLogger import ardLog, ardSave
from streaming import optiTracker
import random

# Initialise kinematics class:
kine = kine()
# Instantiate GUI class
mouseTrack = mouseTracker()
# Instantiate streamer class
optiTrack = optiTracker()

############################################################
# Initialise variables 
SAMP_FREQ = 1/kine.timeStep

flagStop = False

currentX = 0
currentY = 0
# Target must be cast as immutable type (float, in this case) so that 
# the current position doesn't update at same time as target
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
leftLim = kine.sideLength - maxContract # 17 mm contraction wrt LHS vertex
rightLim = kine.sideLength - minContract # 2 mm contraction wrt LHS vertex
initialXFlag = False
randoPosition = False

# Initialise cable length variables at home position
cVolL, cVolR, cVolT = 0, 0, 0
cableL, cableR, cableT = kine.sideLength, kine.sideLength, kine.sideLength
[targetL, targetR, targetT, tJpinv] = kine.cableLengths(targetX, targetY)
realStepL, realStepR, realStepT = 0, 0, 0
angleL, angleR, angleT = 0, 0, 0

# Set current volume (ignore tSpeed and step values) 
[cVolL, tSpeedL, stepL, LcRealL, angleL] = kine.length2Vol(cableL, targetL)
[cVolR, tSpeedR, stepR, LcRealR, angleR] = kine.length2Vol(cableR, targetR)
[cVolT, tSpeedT, stepT, LcRealT, angleT] = kine.length2Vol(cableT, targetT)

[tVolL, vDotL, dDotL, fStepL, stepL, tSpeedL, LcRealL, angleL] = kine.volRate(cVolL, cableL, targetL)
[tVolR, vDotR, dDotR, fStepR, stepR, tSpeedR, LcRealR, angleR] = kine.volRate(cVolR, cableR, targetR)
[tVolT, vDotT, dDotT, fStepT, stepT, tSpeedT, LcRealT, angleT] = kine.volRate(cVolT, cableT, targetT)
[OCRL, OCRR, OCRT, LStep, RStep, TStep] = kine.freqScale(fStepL, fStepR, fStepT)
LStep, RStep, TStep = 0, 0, 0

# Set initial pressure and calibration variables
pressL, pressR, pressT = 0, 0, 0
timeL, timeR, timeT = 0, 0, 0

# Current position
cStepL = stepL
cStepR = stepR
cStepT = stepT
dStepL, dStepR, dStepT  = 0, 0, 0

StepNoL, StepNoR, StepNoT = stepL, stepR, stepT

###############################################################
# Connect to Arduinos
try:
    [lhs, reply] = connect("LHS", 6)
    print(reply)
    [rhs, reply] = connect("RHS", 5)
    print(reply)
    [top, reply] = connect("TOP", 4)
    print(reply)

    #############################################################
    # Calibrate arduinos for zero volume - maintain negative pressure for 4 seconds
    calibL = False
    calibR = False
    calibT = False
    calibration = False
    # Calibration ON if TRUE below:
    while (calibration != False):
        # [realStepL, pressL, timeL] = listenZero(lhs, calibL)
        [realStepR, pressR, timeR] = listenZero(rhs, calibR)
        # [realStepT, pressT, timeT] = listenZero(top, calibT)
        # print(realStepL, pressL)
        print(realStepR, pressR)
        # print(realStepT, pressT)
        # if (realStepL == "0000LHS"):
        #     calibL = True
        if (realStepR == "0000RHS"):
            calibR = True
        # if (realStepT == "0000TOP"):
        #     calibT = True
        if (1 * calibR * 1 == 1):
            calibration = True
            # Send 0s instead of StepNo as signal that calibration done
            ardLog(realStepL, LcRealL, angleL, 0, pressL, timeL,\
                realStepR, LcRealR, angleR, 0, pressR, timeR,\
                realStepT, LcRealT, angleT, 0, pressT, timeT)
        else:
            ardLog(realStepL, LcRealL, angleL, StepNoL, pressL, timeL,\
                realStepR, LcRealR, angleR, StepNoR, pressR, timeR,\
                realStepT, LcRealT, angleT, StepNoT, pressT, timeT)

    ################################################################
    # Begin main loop
    try:
        # Bring up GUI
        mouseTrack.createTracker()
        while(flagStop == False):
            # tick = time.perf_counter()
            [targetX, targetY, tMillis, flagStop] = mouseTrack.iterateTracker()
            tSecs = tMillis/1000

            # Do cable and syringe calculations:
            # Get target lengths and Jacobian from target point

            ##########################################
            # Manually set targets here

            ###
            # Discretise input:
            # targetY = 0
            # targetX = kine.sideLength*round(targetX/(kine.sideLength/10))/10
            ###

            ###
            # Oscillate input between two values
            # Sampling frequency is ~20.8333 Hz, so use this to find speed.
            # if targetXTest >= rightLim:
            #     targetXTest = rightLim
            #     if delayCount < delayLim:
            #         delayCount += 1
            #     else:
            #         toggleDirection = -1
            #         delayCount = 0
            #         targetXTest = targetXTest + toggleDirection*oscStep

            # elif targetXTest <= leftLim:
            #     targetXTest = leftLim
            #     if delayCount < delayLim:
            #         delayCount += 1
            #     else:
            #         toggleDirection = 1
            #         delayCount = 0
            #         targetXTest = targetXTest + toggleDirection*oscStep
            # else:
            #     targetXTest = targetXTest + toggleDirection*oscStep

            ###

            ###
            # Set random target
            if initialXFlag == True:
                if randoPosition == False:
                    targetXTest = random.uniform(leftLim, rightLim)
                    randoPosition = True
                else:
                    targetXTest = currentX
                    if delayCount < delayLim/2:
                        delayCount += 1
                    else:
                        delayCount = 0
                        randoPosition = False
            ###

            ###
            # Initially pause at midpoint
            if initialXFlag == False:
                targetXTest = mouseTrack.xCoord
                if delayCount < delayLim:
                    delayCount += 1
                else:
                    delayCount = 0
                    initialXFlag = True
            ###

            # Ensure 1 decimal place
            targetXTest = round(100*targetXTest)/100 # Will only allow a step oscStep with 2 decimal places
            targetX = targetXTest
            targetY = 0
            # print(targetX)
            #########################################

            # Limit input
            if targetX <= 2:
                targetX = 2
            elif targetX >= 17:
                targetX = 17
            # Add proximity restriction to top vertex as well, where both x = side/2 and y = maxY
                # Cable length restriction - percentage flat length


            [targetL, targetR, targetT, tJpinv] = kine.cableLengths(targetX, targetY)
            # Get cable speeds using Jacobian at current point and calculation of input speed
            # [lhsV, rhsV, topV] = cableSpeeds(currentX, currentY, targetX, targetY, cJpinv, tSecs)
            # Get volumes, volrates, syringe speeds, pulse freq, step counts, & cablespeed estimate for each pump
            [tVolL, vDotL, dDotL, fStepL, stepL, tSpeedL, LcRealL, angleL] = kine.volRate(cVolL, cableL, targetL)
            [tVolR, vDotR, dDotR, fStepR, stepR, tSpeedR, LcRealR, angleR] = kine.volRate(cVolR, cableR, targetR)
            [tVolT, vDotT, dDotT, fStepT, stepT, tSpeedT, LcRealT, angleT] = kine.volRate(cVolT, cableT, targetT)
            # CALCULATE FREQS FROM VALID STEP NUMBER
            # stepL is master position, cStepL is real, speed controlled position.
            dStepL = stepL - cStepL
            dStepR = stepR - cStepR
            dStepT = stepT - cStepT
            fStepL = dStepL*SAMP_FREQ
            fStepR = dStepR*SAMP_FREQ
            fStepT = dStepT*SAMP_FREQ
            [OCRL, OCRR, OCRT, LStep, RStep, TStep] = kine.freqScale(fStepL, fStepR, fStepT)
            StepNoL += LStep
            StepNoR += RStep # RStep = dStepR scaled for speed (w rounding differences)
            StepNoT += TStep
            # Send step number to arduinos:
            sendStep(lhs, StepNoL)
            sendStep(rhs, StepNoR)
            sendStep(top, StepNoT)

            optiTrack.readSocket()
            
            # Update current position, cable lengths, and volumes as previous targets
            cJpinv = tJpinv
            currentX = targetX
            currentY = targetY
            cableL = targetL
            cableR = targetR
            cableT = targetT
            cVolL = tVolL
            cVolR = tVolR
            cVolT = tVolT
            cStepL = StepNoL
            cStepR = StepNoR
            cStepT = StepNoT
            ardLog(realStepL, LcRealL, angleL, StepNoL, pressL, timeL,\
                realStepR, LcRealR, angleR, StepNoR, pressR, timeR,\
                realStepT, LcRealT, angleT, StepNoT, pressT, timeT)

            [realStepL, pressL, timeL] = listenReply(lhs)
            [realStepR, pressR, timeR] = listenReply(rhs)
            [realStepT, pressT, timeT] = listenReply(top)
            # tock = time.perf_counter()
            # elapsed = tock-tick
            # print(f"{elapsed:0.4f}")
            # print("Pressure: ", pressL, pressR, pressT)
            # print("Real Pos: ", realStepL, realStepR, realStepT)
            # print(targetL, targetR, targetT)
            # print(tVolL, tVolR, tVolT)

        flagStop = mouseTrack.closeTracker()

    except ZeroDivisionError:
        pass
    
except KeyboardInterrupt:
    pass

finally:
    ###########################################################################
    # Stop program
    # Disable pumps and set them to idle state
    try:
        if lhs.is_open:
            sendStep(lhs, "Closed")
            [realStepL, pressL, timeL] = listenReply(lhs)
            print(realStepL, pressL, timeL)

        if rhs.is_open:
            sendStep(rhs, "Closed")
            [realStepR, pressR, timeR] = listenReply(rhs)
            print(realStepR, pressR, timeR)
        
        if top.is_open:
            sendStep(top, "Closed")
            [realStepT, pressT, timeT] = listenReply(top)
            print(realStepT, pressT, timeT)
    
            # Save values gathered from arduinos
            ardLog(realStepL, LcRealL, angleL, StepNoL, pressL, timeL,\
                realStepR, LcRealR, angleR, StepNoR, pressR, timeR,\
                realStepT, LcRealT, angleT, StepNoT, pressT, timeT)
            ardSave()

    except NameError:
        [lhs, reply] = connect("LHS", 6)
        print(reply)
        [rhs, reply] = connect("RHS", 5)
        print(reply)
        [top, reply] = connect("TOP", 4)
        print(reply)

    # Close serial connections
    lhs.close()
    rhs.close()
    top.close()

