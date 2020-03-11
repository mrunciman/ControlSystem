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
from kinematics import cableLengths, volRate, cableSpeeds
from mouseGUI import mouseTracker
# import numpy as np
# from numpy import linalg as la
import math as mt

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
currentX = 25
currentY = 14.435
# Target must be cast as immutable type (float, in this case) so that 
# the current position doesn't update at same time as target
targetX = 0.0
targetY = 0.0

#Initialise variables
vDotL, dDotL, fStepL, vCL, vTL, dCL, dTL = 0, 0, 0, 0, 0, 0, 0
vDotR, dDotR, fStepR, vCR, vTR, dCR, dTR = 0, 0, 0, 0, 0, 0, 0
vDotT, dDotT, fStepT, vCT, vTT, dCT, dTT = 0, 0, 0, 0, 0, 0, 0
lhsV, rhsV, topV = 0, 0, 0
targetL, targetR, targetT, tJpinv = 0, 0, 0, 0

#Initialise cable length variables at home position
[cableL, cableR, cableT, cJpinv] = cableLengths(currentX, currentY)
# print("Cable lengths: ", cableL, cableR, cableT)

# Instantiate class that sets up GUI
mouseTrack = mouseTracker()
# First attempt at main loop
mouseTrack.createTracker()
while(1):
    [targetX, targetY, tMillis, flagStop] = mouseTrack.iterateTracker()
    # print("Current: ", currentX, currentY)
    # print("Target:  ", targetX, targetY)
    diffP = mt.sqrt((targetX-currentX)**2 + (targetY-currentY)**2)
    tSecs = tMillis/1000
    # print(diffP, "\n")
    # print(tMillis)
    # FILTER INPUT FROM GUI
        # CHECK FOR ZEROS
        # IF TARGET == CURRENT

    # if diffP = 0:
    try:
        [targetL, targetR, targetT, tJpinv] = cableLengths(targetX, targetY)
        [lhsV, rhsV, topV] = cableSpeeds(currentX, currentY, targetX, targetY, tJpinv, tSecs)
        [vDotL, dDotL, fStepL, vCL, vTL, dCL, dTL] = volRate(cableL, targetL, tSecs)
        [vDotR, dDotR, fStepR, vCR, vTR, dCR, dTR] = volRate(cableR, targetR, tSecs)
        [vDotT, dDotT, fStepT, vCT, vTT, dCT, dTT] = volRate(cableT, targetT, tSecs)
    except ZeroDivisionError as e:
        pass
        ####
        # SEND VALUES TO ARDUINOS
        ###
    # print("Cable lengths: ", targetL, targetR, targetT, tSecs)
    # print("Volume rate, syringe speed and pulse freq: ", vDotL, dDotL, fStepL)
    # print("Frequencies: ", fStepL, fStepR, fStepT)
    print("Cable speeds: ", lhsV, rhsV, topV)

    # Update current position and cable lengths as previous targets
    currentX = targetX
    currentY = targetY
    cableL = targetL
    cableR = targetR
    cableT = targetT
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
# rhs.close()

