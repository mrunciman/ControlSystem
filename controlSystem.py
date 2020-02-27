from arduinoInterface import connect, listenSteps, listenPress
from kinematics import cableLengths, length2Vol

# top = connect("TOP", 4)
# lhs = connect("LHS", 5)
# rhs = connect("RHS", 6)

# i = 0
# while i < 10:
#     rhsSteps = listenSteps(rhs)
#     rhsPressure = listenPress(rhs)
#     print(rhsSteps)
#     i+=1

[cableL, cableR, cableT] = cableLengths(25, 14.43)
print(cableL, cableR, cableT)

volume = length2Vol(cableT)
print(volume)

# top.close()
# lhs.close()
# rhs.close()

