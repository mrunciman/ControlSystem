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

from arduinoInterface import connect, sendStep, sendOCR
from kinematics import cableLengths, volRate, freq2OCR
from mouseGUI import mouseTracker
# import numpy as np
# from numpy import linalg as la
# import math as mt

tHome = 4 # Time in s to go to home position from 0 (flat actuators)

lhs= "Kegs"
# rhs = "Friel"
top = "Kinloch"

try:
    # lhs = connect("LHS", 5)
    [rhs, reply] = connect("RHS", 4)
    print(reply)
    # top = connect("TOP", 4)
except KeyboardInterrupt:
    rhs.close()

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

[tVolL, vDotL, dDotL, fStepL, stepL, OCRL] = volRate(cVolL, cableL, targetL)
[tVolR, vDotR, dDotR, fStepR, stepR, OCRR] = volRate(cVolR, cableR, targetR)
[tVolT, vDotT, dDotT, fStepT, stepT, OCRT] = volRate(cVolT, cableT, targetT)
[OCRL, OCRR, OCRT] = freq2OCR(fStepL, fStepR, fStepT)

fHomeL = stepL/tHome
fHomeR = stepR/tHome
fHomeT = stepT/tHome


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
        [tVolL, vDotL, dDotL, fStepL, stepL, OCRL] = volRate(cVolL, cableL, targetL)
        [tVolR, vDotR, dDotR, fStepR, stepR, OCRR] = volRate(cVolR, cableR, targetR)
        [tVolT, vDotT, dDotT, fStepT, stepT, OCRT] = volRate(cVolT, cableT, targetT)
        # Calculate compare register values that produce closest frequencies
        [OCRL, OCRR, OCRT] = freq2OCR(fStepL, fStepR, fStepT)
        # Send step number to arduinos:
        # realStepL = sendStep(lhs, stepL)
        realStepR = sendStep(rhs, stepR)
        # realStepT = sendStep(top, stepT)
        # Send interrupt register values to arduinos   
        # mL = sendOCR(lhs, OCRL)
        mR = sendOCR(rhs, OCRR)
        # mT = sendOCR(top, OCRT)
        print("Arduino says: ", realStepR, "   Master says: ", stepR)
        print("StepError: ", mR)
    except ZeroDivisionError as e:
        pass
    # print("Cable lengths: ", targetL, targetR, targetT, tSecs)
    # print("Cable speeds: ", lhsV, rhsV, topV)
    # print("Approx speeds: ", speedL, speedR, speedT)
    # print("Volume rates: ", vDotL, vDotR, vDotT)
    # print("Syringe speeds: ", dDotL, dDotR, dDotT)
    # print("Frequencies: ", fStepL, fStepR, fStepT)
    # print("Step Number: ", stepL, stepR, stepT)

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

flagStop = mouseTrack.closeTracker()
# realStepL = sendStep(lhs, "Closed")
realStepR = sendStep(rhs, "Closed")
# realStepT = sendStep(top, "Closed")
# mL = sendOCR(lhs, 0)
# mR = sendOCR(rhs, 0)
# mT = sendOCR(top, 0)
# lhs.close()
rhs.close()
# top.close()


