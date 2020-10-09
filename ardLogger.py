
import time
import csv
import os

# Create a file in 'log' directory and empty contents (if it already exists)
# Append timestamp in ms to name
# location = "Imperial College London/Imperial/Fluidic Control/ControlSystem/logs/pumps"
location = os.path.dirname(__file__)
logTime = time.strftime("%Y-%m-%d %H-%M-%S")
relative = "logs/pumps/arduinoLogs " + logTime + ".csv"
fileName = os.path.join(location, relative) # USE THIS IN REAL TESTS
# fileName = 'ardLogFile.csv' # For test purposes
with open(fileName, mode ='w', newline='') as arduinoLog1: 
    ardLog1 = csv.writer(arduinoLog1)
    ardLog1.writerow(['S_LHS', 'Lc_LHS', 'A_LHS', 'M_LHS', 'P_LHS', 'T_LHS',\
        'S_RHS', 'Lc_RHS', 'A_RHS', 'M_RHS', 'P_RHS', 'T_RHS',\
        'S_TOP', 'Lc_TOP','A_TOP', 'M_TOP', 'P_TOP', 'T_TOP',\
        time.time()])



class ardLogger():

    def __init__(self):
        self.ardData = []
        self.numRows = 0

    def ardLog(self,lhsS, lhsLc, lhsA, lhsMaster, lhsP, lhsT,\
        rhsS, rhsLc, rhsA, rhsMaster, rhsP, rhsT,\
        topS, topLc, topA, topMaster, topP, topT):
        """
        Save stepCount, master cable lengths, pressure values and time 
        from pumps in a list to later save in csv.
        """
        args = locals() # Dictionary of input arguments
        args.pop('self')
        self.ardData.append([i for i in args.values()])
        # print(self.ardData[self.numRows])
        # self.numRows = self.numRows + 1

        # ardData.append([lhsS] + [lhsLc] + [lhsA] + [lhsMaster] + [lhsP] + [lhsT]\
        #     + [rhsS] + [rhsLc] + [rhsA] + [rhsMaster] + [rhsP] + [rhsT]\
        #     + [topS] + [topLc] + [topA] + [topMaster] + [topP] + [topT])
        return


    def ardSave(self):
        """
        Save ardLog list into csv file
        """
        with open(fileName, 'a', newline='') as arduinoLog2:
            ardLog2 = csv.writer(arduinoLog2)
            for i in range(len(self.ardData)):
                ardLog2.writerow(self.ardData[i])
        return