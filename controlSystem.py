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

from arduinoInterface import connect, listenSteps, listenPress, sendFreq, sendOCR
from kinematics import cableLengths, length2Vol, volRate, freq2OCR, cableSpeeds
from mouseGUI import mouseTracker
# import numpy as np
# from numpy import linalg as la
# import math as mt

lhs= "Kegs"
# rhs = "Friel"
top = "Kinloch"

# lhs = connect("LHS", 5)
try:
    rhs = connect("RHS", 4)
except KeyboardInterrupt:
    rhs.close()
# top = connect("TOP", 4)

#Initial values
currentX = 25.0
currentY = 14.435
# Target must be cast as immutable type (float, in this case) so that 
# the current position doesn't update at same time as target
targetX = 0.0
targetY = 0.0

#Initialise variables
vDotL, dDotL, fStepL, stepL, speedL = 0, 0, 0, 0, 0
vDotR, dDotR, fStepR, stepR, speedR = 0, 0, 0, 0, 0
vDotT, dDotT, fStepT, stepT, speedT = 0, 0, 0, 0, 0
lhsV, rhsV, topV = 0, 0, 0
targetL, targetR, targetT, tJpinv = 0, 0, 0, 0

#Initialise cable length variables at home position
[cableL, cableR, cableT, cJpinv] = cableLengths(currentX, currentY)
# currentY = 0
[cVolL, cDL, stepL] = length2Vol(cableL, cableL)
[cVolR, cDR, stepR] = length2Vol(cableR, cableR)
[cVolT, cDT, stepT] = length2Vol(cableT, cableT)
# print("Cable lengths: ", cableL, cableR, cableT)

# Instantiate class that sets up GUI
mouseTrack = mouseTracker()
# First attempt at main loop
mouseTrack.createTracker()
while(1):
    [targetX, targetY, tMillis, flagStop] = mouseTrack.iterateTracker()
    # targetY = 0
    # print("Current: ", currentX, currentY)
    # print("Target:  ", targetX, targetY)
    # diffP = mt.sqrt((targetX-currentX)**2 + (targetY-currentY)**2)
    tSecs = tMillis/1000
    # print(diffP, "\n")
    # print(mouseTrack.xCoord, mouseTrack.yCoord)

    try:
        # Do cable and syringe calculations:
        # Get target lengths and Jacobian from target point
        [targetL, targetR, targetT, tJpinv] = cableLengths(targetX, targetY)
        # Get cable speeds using Jacobian at current point and calculation of input speed
        [lhsV, rhsV, topV] = cableSpeeds(currentX, currentY, targetX, targetY, cJpinv, tSecs)
        # Get volumes, volrates, syringe speeds, pulse freq, step counts, & cablespeed estimate for each pump
        [tVolL, vDotL, dDotL, fStepL, stepL, OCRL] = volRate(cVolL, cableL, targetL)
        [tVolR, vDotR, dDotR, fStepR, stepR, OCRR] = volRate(cVolR, cableR, targetR)
        [tVolT, vDotT, dDotT, fStepT, stepT, OCRT] = volRate(cVolT, cableT, targetT)
        # Calculate compare register values that produce closest frequencies
        [OCRL, OCRR, OCRT] = freq2OCR(fStepL, fStepR, fStepT)
        # Send interrupt register values to arduinos        
        # mL = sendOCR(lhs, OCRL)
        mR = sendOCR(rhs, OCRR)
        # mT = sendOCR(top, OCRT)
        print(mR)
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
    if flagStop:
        break


# currentP = [25, 14.435]
# targetP = [25.5, 14.5]
# [cableL, cableR, cableT, cJpinv] = cableLengths(currentP[0], currentP[1])
# [targetL, targetR, targetT, tJpinv] = cableLengths(targetP[0], targetP[1])
# print("Cable lengths: ", cableL, cableR, cableT)

# # vDot is volume rate in mm3/s, dDot is syringe speed in mm/s, 
# # fStep is step frequency to move at dDot mm/s, vC is current volume
# # vT is target volume, dC and dT are current and target syringe displacements
# secsTime = 0.01632
# [vDotL, dDotL, fStepL, vCL, vTL, dCL, dTL] = volRate(cableL, targetL, secsTime)
# [vDotR, dDotR, fStepR, vCR, vTR, dCR, dTR] = volRate(cableR, targetR, secsTime)
# [vDotT, dDotT, fStepT, vCT, vTT, dCT, dTT] = volRate(cableT, targetT, secsTime)
# print("Volume rate, syringe speed and pulse freq: \n", vDotL, dDotL, fStepL) #vCL, vTL)
# print(vDotR, dDotR, fStepR) #vCR, vTR)
# print(vDotT, dDotT, fStepT) #vCT, vTT)

# [lhsV, rhsV, topV] = cableSpeeds(currentP[0], currentP[1], targetP[0], targetP[1], tJpinv, secsTime)
# print("Cable speeds: ", lhsV, rhsV, topV)



# top.close()
# lhs.close()
rhs.close()

