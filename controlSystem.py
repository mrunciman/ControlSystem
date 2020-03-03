"""
Read in new position from master
Calculate cable lengths at target.
Calcuate target volumes at target.
Calculate desired velocity to reach position in given time
    Use Time equal to reciprocal of master sampling frequency
Calculate rate of cable length change
    Use J and pseudoinverse of J
Calculate rate of volume change (volume flow rate)
Calculate speed of each pump piston
Set step frequency of individual pumps to alter speed
"""

from arduinoInterface import connect, listenSteps, listenPress
from kinematics import cableLengths, length2Vol, cableSpeeds

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
currentP = [25, 0]
targetP = [26, 0]


[cableL, cableR, cableT, Jpinv] = cableLengths(currentP[0], currentP[1])
print(cableL, cableR, cableT)
[volumeL, linearL] = length2Vol(cableL)
[volumeR, linearR] = length2Vol(cableR)
[volumeT, linearT] = length2Vol(cableT)
print(volumeL, linearL)
print(volumeR, linearR)
print(volumeT, linearT)

[lhsV, rhsV, topV] = cableSpeeds(currentP[0], currentP[1], targetP[0], targetP[1], Jpinv)
print(lhsV, rhsV, topV)


# top.close()
# lhs.close()
# rhs.close()

