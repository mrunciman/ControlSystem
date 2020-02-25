from arduinoInterface import connect, listenSteps, listenPress
from kinematics import cableLengths

# top = connect("TOP", 4)
# lhs = connect("LHS", 5)
# rhs = connect("RHS", 6)

# i = 0
# while i < 10:
#     rhsSteps = listenSteps(rhs)
#     rhsPressure = listenPress(rhs)
#     print(rhsSteps)
#     i+=1

[cableL, cableR, cableT] = cableLengths(15, 8.6603)
print(cableL, cableR, cableT)

# top.close()
# lhs.close()
# rhs.close()
