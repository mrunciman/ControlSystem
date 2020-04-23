
import time
import csv

# Create a file in 'log' directory and empty contents (if it already exists)
# Append timestamp in ms to name
location = "C:/Users/msrun/OneDrive - Imperial College London/Imperial/Fluidic Control/ControlSystem/logs/pumps"
logTime = time.time()
fileName = location + "/arduinoLogs" + str(int(logTime*1000)) + ".csv" # USE THIS IN REAL TESTS
fileName = 'ardLog.csv' # For test purposes
with open(fileName, mode ='w', newline='') as arduinoLog1: 
    ardLog1 = csv.writer(arduinoLog1)
    ardLog1.writerow(['S_LHS','P_LHS', 'S_RHS', 'P_RHS', 'S_TOP', 'P_TOP'])

ardData = []


def ardLog(lhsS, lhsP, rhsS, rhsP, topS, topP):
    """
    Save stepCount and pressure values from pumps in a list to later save in csv.
    """
    ardData.append([lhsS] + [lhsP] + [rhsS] + [rhsP] + [topS] + [topP])
    return

def ardSave():
    """
    Save ardLog list into csv file
    """
    with open(fileName, 'a', newline='') as arduinoLog2:
        ardLog2 = csv.writer(arduinoLog2)
        ardLog2.writerows(ardData)
    return