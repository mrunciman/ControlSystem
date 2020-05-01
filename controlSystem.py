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

from arduinoInterface import connect, listenStepPress
from kinematics import cableLengths, volRate, freqScale, length2Vol
from mouseGUI import mouseTracker

tHome = 4 # Time in s to go to home position from 0 (flat actuators)
SAMP_FREQ = 100
# lhs= "Kegs"
# rhs = "Friel"
# top = "Kinloch"

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

flagStop = False
#Initial values
currentX = 0
currentY = 0
# Target must be cast as immutable type (float, in this case) so that 
# the current position doesn't update at same time as target
targetX = 25.0
targetY = 14.435

#Initialise cable length variables at home position
cVolL, cVolR, cVolT = 0, 0, 0
cableL, cableR, cableT = 50, 50, 50
[targetL, targetR, targetT, tJpinv] = cableLengths(targetX, targetY)
realStepL, realStepR, realStepT = 0, 0, 0

#Set current volume (ignore tSpeed and step values) 
[cVolL, tSpeedL, stepL] = length2Vol(cableL, targetL)
[cVolR, tSpeedR, stepR] = length2Vol(cableR, targetR)
[cVolT, tSpeedT, stepT] = length2Vol(cableT, targetT)

[tVolL, vDotL, dDotL, fStepL, stepL, tSpeedL] = volRate(cVolL, cableL, targetL)
[tVolR, vDotR, dDotR, fStepR, stepR, tSpeedR] = volRate(cVolR, cableR, targetR)
[tVolT, vDotT, dDotT, fStepT, stepT, tSpeedT] = volRate(cVolT, cableT, targetT)
[OCRL, OCRR, OCRT, LStep, RStep, TStep] = freqScale(fStepL, fStepR, fStepT)
LStep, RStep, TStep = 0, 0, 0
fHomeL = stepL/tHome
fHomeR = stepR/tHome
fHomeT = stepT/tHome

cStepL = stepL
cStepR = stepR
cStepT = stepT
dStepL, dStepR, dStepT  = 0, 0, 0

SteppyL, SteppyR, SteppyT = 2168, 2168, 2168
# Instantiate class that sets up GUI
mouseTrack = mouseTracker()
# First attempt at main loop
mouseTrack.createTracker()
while(flagStop == False):
    [targetX, targetY, tMillis, flagStop] = mouseTrack.iterateTracker()
    tSecs = tMillis/1000
    try:
        # Do cable and syringe calculations:
        # Get target lengths and Jacobian from target point
        [targetL, targetR, targetT, tJpinv] = cableLengths(targetX, targetY)
        # Get cable speeds using Jacobian at current point and calculation of input speed
        # [lhsV, rhsV, topV] = cableSpeeds(currentX, currentY, targetX, targetY, cJpinv, tSecs)
        # Get volumes, volrates, syringe speeds, pulse freq, step counts, & cablespeed estimate for each pump
        [tVolL, vDotL, dDotL, fStepL, stepL, tSpeedL] = volRate(cVolL, cableL, targetL)
        [tVolR, vDotR, dDotR, fStepR, stepR, tSpeedR] = volRate(cVolR, cableR, targetR)
        [tVolT, vDotT, dDotT, fStepT, stepT, tSpeedT] = volRate(cVolT, cableT, targetT)
        # CALCULATE FREQS FROM VALID STEP NUMBER
        # stepL is master position, cStepL is real, speed controlled position.
        dStepL = stepL - cStepL 
        dStepR = stepR - cStepR
        dStepT = stepT - cStepT
        fStepL = dStepL*SAMP_FREQ
        fStepR = dStepR*SAMP_FREQ
        fStepT = dStepT*SAMP_FREQ
        [OCRL, OCRR, OCRT, LStep, RStep, TStep] = freqScale(fStepL, fStepR, fStepT)
        SteppyL += LStep
        SteppyR += RStep # RStep = dStepR scaled for speed (w rounding differences)
        SteppyT += TStep
        # Send step number to arduinos:
        [realStepL, lhsP] = listenStepPress(lhs, SteppyL)
        [realStepR, rhsP] = listenStepPress(rhs, SteppyR)
        [realStepT, topP] = listenStepPress(top, SteppyT)


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
    cStepL = SteppyL
    cStepR = SteppyR
    cStepT = SteppyT
    # print("Cable lengths: ", targetL, targetR, targetT, tSecs)
    # print("Cable speeds: ", lhsV, rhsV, topV)
    # print("Approx speeds: ", speedL, speedR, speedT)
    # print("Volume rates: ", vDotL, vDotR, vDotT)
    # print("Syringe speeds: ", dDotL, dDotR, dDotT)
    # print("Frequencies: ", fStepL, fStepR, fStepT)
    # print("Step Number: ", stepL, stepR, stepT)

    # print("Arduino says: ", realStepR, "   Master says: ", stepR)
    # print("Master says: ", SteppyT)
    print("Pressure: ", lhsP, rhsP, topP, "  StepT: ", realStepT)

flagStop = mouseTrack.closeTracker()
[realStepL, lhsP] = listenStepPress(lhs, "Closed")
[realStepR, rhsP] = listenStepPress(rhs, "Closed")
[realStepT, topP] = listenStepPress(top, "Closed")
print(realStepL)
print(realStepR)
print(realStepT)

lhs.close()
rhs.close()
top.close()


