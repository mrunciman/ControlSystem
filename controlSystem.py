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

from arduinoInterface import connect, listenStepPress, listenZero
from kinematics import kine #cableLengths, volRate, freqScale, length2Vol
from mouseGUI import mouseTracker
from ardLogger import ardLog, ardSave

# Initialise kinematics class:
kine = kine()
# Instantiate GUI class
mouseTrack = mouseTracker()

############################################################
# Initialise variables 
SAMP_FREQ = 1/kine.timeStep

flagStop = False

currentX = 0
currentY = 0
# Target must be cast as immutable type (float, in this case) so that 
# the current position doesn't update at same time as target
targetX = mouseTrack.xCoord
targetY = mouseTrack.yCoord

# Initialise cable length variables at home position
cVolL, cVolR, cVolT = 0, 0, 0
cableL, cableR, cableT = kine.sideLength, kine.sideLength, kine.sideLength
[targetL, targetR, targetT, tJpinv] = kine.cableLengths(targetX, targetY)
realStepL, realStepR, realStepT = 0, 0, 0

# Set current volume (ignore tSpeed and step values) 
[cVolL, tSpeedL, stepL, LcRealL] = kine.length2Vol(cableL, targetL)
[cVolR, tSpeedR, stepR, LcRealR] = kine.length2Vol(cableR, targetR)
[cVolT, tSpeedT, stepT, LcRealT] = kine.length2Vol(cableT, targetT)

[tVolL, vDotL, dDotL, fStepL, stepL, tSpeedL, LcRealL] = kine.volRate(cVolL, cableL, targetL)
[tVolR, vDotR, dDotR, fStepR, stepR, tSpeedR, LcRealR] = kine.volRate(cVolR, cableR, targetR)
[tVolT, vDotT, dDotT, fStepT, stepT, tSpeedT, LcRealT] = kine.volRate(cVolT, cableT, targetT)
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
    [rhs, reply] = connect("RHS", 4)
    print(reply)
    [top, reply] = connect("TOP", 5)
    print(reply)
except KeyboardInterrupt:
    lhs.close()
    rhs.close()
    top.close()

#############################################################
# Calibrate arduinos for zero volume - maintain negative pressure for 5 seconds
calibL = False
calibR = False
calibT = False
calibration = False
# Calibration ON if TRUE below:
while (calibration != False):
    [realStepL, pressL, timeL] = listenZero(lhs, calibL)
    # [realStepR, pressR, timeR] = listenZero(rhs, calibR)
    # [realStepT, pressT, timeT] = listenZero(top, calibT)
    print(realStepL, pressL)
    # print(realStepR, pressR)
    # print(realStepT, pressT)
    ardLog(realStepL, LcRealL, StepNoL, pressL, timeL, realStepR, LcRealR, StepNoR, pressR, timeR, realStepT, LcRealT, StepNoT, pressT, timeT)
    if (realStepL == "0000LHS"):
        calibL = True
    # if (realStepR == "0000RHS"):
    #     calibR = True
    # if (realStepT == "0000TOP"):
    #     calibT = True
    if (calibL * 1 * 1 == 1):
        calibration = True


################################################################
# Begin main loop

# Bring up GUI
mouseTrack.createTracker()

while(flagStop == False):
    [targetX, targetY, tMillis, flagStop] = mouseTrack.iterateTracker()
    tSecs = tMillis/1000
    try:
        # Do cable and syringe calculations:
        # Get target lengths and Jacobian from target point
        
        # Manually set targets here
        targetY = 0
        targetX = kine.sideLength*round(targetX/(kine.sideLength/10))/10
        if targetX <= kine.sideLength*0.1:
            targetX = kine.sideLength*0.1
        elif targetX >= kine.sideLength*0.9:
            targetX = kine.sideLength*0.9
        print(targetX)

        [targetL, targetR, targetT, tJpinv] = kine.cableLengths(targetX, targetY)
        # Get cable speeds using Jacobian at current point and calculation of input speed
        # [lhsV, rhsV, topV] = cableSpeeds(currentX, currentY, targetX, targetY, cJpinv, tSecs)
        # Get volumes, volrates, syringe speeds, pulse freq, step counts, & cablespeed estimate for each pump
        [tVolL, vDotL, dDotL, fStepL, stepL, tSpeedL, LcRealL] = kine.volRate(cVolL, cableL, targetL)
        [tVolR, vDotR, dDotR, fStepR, stepR, tSpeedR, LcRealR] = kine.volRate(cVolR, cableR, targetR)
        [tVolT, vDotT, dDotT, fStepT, stepT, tSpeedT, LcRealT] = kine.volRate(cVolT, cableT, targetT)
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
        [realStepL, pressL, timeL] = listenStepPress(lhs, StepNoL)
        [realStepR, pressR, timeR] = listenStepPress(rhs, StepNoR)
        [realStepT, pressT, timeT] = listenStepPress(top, StepNoT)


    except ZeroDivisionError as e:
        pass

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
    ardLog(realStepL, LcRealL, StepNoL, pressL, timeL, realStepR, LcRealR, StepNoR, pressR, timeR, realStepT, LcRealT, StepNoT, pressT, timeT)

    # print("Pressure: ", pressL, pressR, pressT)
    # print("Real Pos: ", realStepL, realStepR, realStepT)


###########################################################################
# Stop program

# Disable pumps and set them to idle state
flagStop = mouseTrack.closeTracker()
[realStepL, pressL, timeL] = listenStepPress(lhs, "Closed")
[realStepR, pressR, timeR] = listenStepPress(rhs, "Closed")
[realStepT, pressT, timeT] = listenStepPress(top, "Closed")
print(realStepL, pressL, timeL)
print(realStepR, pressR, timeR)
print(realStepT, pressT, timeT)

# Save values gathered from arduinos
ardLog(realStepL, LcRealL, StepNoL, pressL, timeL, realStepR, LcRealR, StepNoR, pressR, timeR, realStepT, LcRealT, StepNoT, pressT, timeT)
ardSave()

# Close serial connections
lhs.close()
rhs.close()
top.close()


