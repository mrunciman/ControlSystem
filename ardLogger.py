
import time
import csv

# Create a file in 'log' directory and empty contents (if it already exists)
# Append timestamp in ms to name
location = "C:/Users/msrun/OneDrive - Imperial College London/Imperial/Fluidic Control/ControlSystem/logs/pumps"
# logTime = time.time()
logTime = time.strftime("%Y-%m-%d-_-%H-%M-%S")
fileName = location + "/arduinoLogs " + logTime + ".csv" # USE THIS IN REAL TESTS
# fileName = 'ardLogFile.csv' # For test purposes
with open(fileName, mode ='w', newline='') as arduinoLog1: 
    ardLog1 = csv.writer(arduinoLog1)
    ardLog1.writerow(['S_LHS', 'M_LHS', 'P_LHS', 'T_LHS', 'S_RHS', 'M_RHS', 'P_RHS', 'T_RHS', 'S_TOP', 'M_TOP', 'P_TOP', 'T_TOP'])

ardData = []


def ardLog(lhsS, lhsMaster, lhsP, lhsT, rhsS, rhsMaster, rhsP, rhsT, topS, topMaster, topP, topT):
    """
    Save stepCount, master cable lengths, pressure values and time 
    from pumps in a list to later save in csv.
    """
    ardData.append([lhsS] + [lhsMaster] + [lhsP] + [lhsT] + [rhsS] + [rhsMaster] + [rhsP] + [rhsT] + [topS] + [topMaster] + [topP] + [topT])
    return

def ardSave():
    """
    Save ardLog list into csv file
    """
    with open(fileName, 'a', newline='') as arduinoLog2:
        ardLog2 = csv.writer(arduinoLog2)
        ardLog2.writerows(ardData)
    return