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

from arduinoInterface import connect, listenSteps, listenPress
from kinematics import cableLengths, length2Vol, volRate, cableSpeeds
from mouseGUI import createTracker, iterateTracker
import numpy as np
from numpy import linalg as la
import time

# top = connect("TOP", 4)
# lhs = connect("LHS", 5)
# rhs = connect("RHS", 6)

# i = 0
# while i < 10:
#     rhsSteps = listenSteps(rhs)
#     rhsPressure = listenPress(rhs)
#     print(rhsSteps)
#     i+=1

#Initial values
currentP = [25, 14.435]
# currentP = [25, 13]
targetP = [25.5, 13.5]
[cableL, cableR, cableT, cJpinv] = cableLengths(currentP[0], currentP[1])
print("Cable lengths 1: ", cableL, cableR, cableT)
# First attempt at main loop
createTracker()
while(1):
    [targetP[0], targetP[1], tMillis, flagStop] = iterateTracker()
    # diffP = mt.sqrt((targetP[0]-currentP[0])**2 + (targetP[1]-currentP[1])**2)
    # diffP = 1000000*diffP
    # deltaP = targetP-currentP
    diffP = la.norm(np.array([[targetP[0]-currentP[0]], [targetP[1]-currentP[1]]]))
    tSecs = tMillis/1000
    # ADAPT KINEMATICS TO HANDLE VARYING TIME INPUT IN ms
    # print(diffP)
    # print(tSecs)
    # FILTER INPUT FROM GUI
        # CHECK FOR ZEROS
        # IF TARGET == CURRENT
    if diffP >= 0.1:
        [targetL, targetR, targetT, tJpinv] = cableLengths(targetP[0], targetP[1])
        [vDotL, dDotL, fStepL, vCL, vTL, dCL, dTL] = volRate(cableL, targetL, tSecs)
        [vDotR, dDotR, fStepR, vCR, vTR, dCR, dTR] = volRate(cableR, targetR, tSecs)
        [vDotT, dDotT, fStepT, vCT, vTT, dCT, dTT] = volRate(cableT, targetT, tSecs)
        [lhsV, rhsV, topV] = cableSpeeds(currentP[0], currentP[1], targetP[0], targetP[1], tJpinv, tSecs)
    # print("Cable lengths: ", targetL, targetR, targetT, tSecs)
    # print("Volume rate, syringe speed and pulse freq: \n", vDotL, dDotL, fStepL)
    currentP = targetP
    time.sleep(0.1)
    if flagStop:
        break



[cableL, cableR, cableT, cJpinv] = cableLengths(currentP[0], currentP[1])
[targetL, targetR, targetT, tJpinv] = cableLengths(targetP[0], targetP[1])
print("Cable lengths: ", cableL, cableR, cableT)

# vDot is volume rate in mm3/s, dDot is syringe speed in mm/s, 
# fStep is step frequency to move at dDot mm/s, vC is current volume
# vT is target volume, dC and dT are current and target syringe displacements
secsTime = 0.01632
[vDotL, dDotL, fStepL, vCL, vTL, dCL, dTL] = volRate(cableL, targetL, secsTime)
[vDotR, dDotR, fStepR, vCR, vTR, dCR, dTR] = volRate(cableR, targetR, secsTime)
[vDotT, dDotT, fStepT, vCT, vTT, dCT, dTT] = volRate(cableT, targetT, secsTime)
print("Volume rate, syringe speed and pulse freq: \n", vDotL, dDotL, fStepL) #vCL, vTL)
print(vDotR, dDotR, fStepR) #vCR, vTR)
print(vDotT, dDotT, fStepT) #vCT, vTT)

[lhsV, rhsV, topV] = cableSpeeds(currentP[0], currentP[1], targetP[0], targetP[1], tJpinv, secsTime)
print("Cable speeds: ", lhsV, rhsV, topV)



# top.close()
# lhs.close()
# rhs.close()

