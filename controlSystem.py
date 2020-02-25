from arduinoInterface import connect, listenSteps, listenPress
import time

top = connect("TOP", 4)
lhs = connect("LHS", 5)
rhs = connect("RHS", 6)

i = 0
while i < 10:
    rhsSteps = listenSteps(rhs)
    rhsPressure = listenPress(rhs)
    print(rhsSteps)
    i+=1


top.close()
lhs.close()
rhs.close()
print(top.is_open)
print(lhs.is_open)
print(rhs.is_open)