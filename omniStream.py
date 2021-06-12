#!/usr/bin/env python3

import socket
import time
# import subprocess
# from ctypes import *



class omniStreamer():
    def __init__(self):
        self.server_addr = ('localhost', 8888)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.omniX = 0.0 # mm
        self.omniY = 0.0
        self.omniZ = 0.0

    def connectOmni(self):
        # problem with this is that it wiats for program to terminate, which never happens
        # omniServer = subprocess.run([r"C:\OpenHaptics\Developer\3.5.0\examples\HD\console\HelloHapticDevice\Win32\Debug\HelloHapticDevice.exe"],\
        #     check=True, capture_output=True, text=True)
        # print("stdout:", omniServer.stdout)
        # print("stdout:", omniServer.stderr)

        try:
            self.sock.connect(self.server_addr)
            # self.sock.setblocking(0)
            self.sock.settimeout(0.1)
            print("Connected to {:s}".format(repr(self.server_addr)))
        except AttributeError as ae:
            print("Error creating the socket: {}".format(ae))

    def getOmniCoords(self):
        try:
            handshake = b'1'
            self.sock.send(handshake)
            data = self.sock.recv(512)
            # print(len(data))
            stringdata = data.decode('utf-8')
            numdata = stringdata.split(";")
            numdata = numdata[0:-1] # Remove empty entry at end due to split
            # print(numdata)
            # numdata has an empty character appended to it as the last element
            # if (len(numdata) > 3): # Need a better filter here - start and end bits?
            count = 1
            for i in numdata:
                if i == "S":
                    if len(numdata)-count >= 4:
                        if numdata[count+3] == "E": #index is count minus one (then plus x for relevant digit), for 0 indexing
                            self.omniX = numdata[count]
                            self.omniY = numdata[count+1]
                            self.omniZ = numdata[count+2]
                            # print("x: ", self.omniX, ", y: ", self.omniY, ", z: ", self.omniZ)
                            break
                        else:
                            # print(numdata[count+3])
                            break # just ignore and wait for nex one
                            # raise ValueError('Somehow the endByte is not what was expected') # Somehow the endByte isn't what was expected
                else:
                    count+=1

            # if numdata[0] == ord("S"):
            #     self.omniX = float(numdata[0])
            #     self.omniY = float(numdata[1])
            #     self.omniZ = float(numdata[2])
                # print("x: ", self.omniX, ", y: ", self.omniY, ", z: ", self.omniZ)
        except socket.timeout:
            pass
        except socket.error as se:
            print("Exception on socket: {}".format(se))
            print("Closing socket")
            self.sock.close()
        # return self.omniX, self.omniY, self.omniZ

    def omniMap(self):#, xFromOmni, yFromOmni, zFromOmni):
        #Calibrated position in inkwell:
        #  x:  0.00000 , y:  -65.51071 , z:  -88.11420
        xMapped = abs(float(self.omniX)) # abs jsut for test
        yMapped = abs(float(self.omniY))
        zMapped = -float(self.omniZ) # Omni direction is opposite to real direction
        print("x: ", xMapped, ", y: ", yMapped, ", z: ", zMapped)
        return float(xMapped), float(yMapped), float(zMapped)
        
        

    def omniClose(self):
        self.sock.close()


# if __name__ == "__main__":
#     main()