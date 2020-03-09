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

# top = connect("TOP", 4)
# lhs = connect("LHS", 5)
# rhs = connect("RHS", 6)

# i = 0
# while i < 10:
#     rhsSteps = listenSteps(rhs)
#     rhsPressure = listenPress(rhs)
#     print(rhsSteps)
#     i+=1

# currentP = [25, 14.435]
currentP = [25, 13]
targetP = [25.5, 13.5]


[cableL, cableR, cableT, cJpinv] = cableLengths(currentP[0], currentP[1])
[targetL, targetR, targetT, tJpinv] = cableLengths(targetP[0], targetP[1])
print("Cable lengths: ", cableL, cableR, cableT)

# vDot is volume rate in mm3/s, dDot is syringe speed in mm/s, 
# fStep is step frequency to move at dDot mm/s, vC is current volume
# vT is target volume, dC and dT are current and target syringe displacements
[vDotL, dDotL, fStepL, vCL, vTL, dCL, dTL] = volRate(cableL, targetL)
[vDotR, dDotR, fStepR, vCR, vTR, dCR, dTR] = volRate(cableR, targetR)
[vDotT, dDotT, fStepT, vCT, vTT, dCT, dTT] = volRate(cableT, targetT)
print("Volume rate, syringe speed and pulse freq: \n", vDotL, dDotL, fStepL) #vCL, vTL)
print(vDotR, dDotR, fStepR) #vCR, vTR)
print(vDotT, dDotT, fStepT) #vCT, vTT)

[lhsV, rhsV, topV] = cableSpeeds(currentP[0], currentP[1], targetP[0], targetP[1], tJpinv)
print("Cable speeds: ", lhsV, rhsV, topV)



# top.close()
# lhs.close()
# rhs.close()

