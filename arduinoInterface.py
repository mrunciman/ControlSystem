"""
Set of functions to facilitate connection to syringe pumps, which are controlled by arduinos.
Arduinos connected to a USB hub so COM port names must be checked in main code.
Enables connection, pressure monitoring, and step count monitoring for later burst
protection and volume calculation.
"""
import serial 
import time

class ardInterfacer:

    def __init__(self, namePump, numPort):
        self.pumpName = namePump
        self.portNumber = numPort
        self.ser = serial.Serial()
        self.ser.port = 'COM%s' % (self.portNumber) 
        self.ser.baudrate = 115200
        self.ser.timeout = 0
        self.ser.open()
    

    def connect(self):
        """
        Each pump requires verification of the cable it is contolling (top, lhs, or rhs).
        Pumps will function normally when the pumpName sent here matches the name hardcoded 
        on each arduino.
        e.g. top = connect("TOP", 4)
        """
        message = self.pumpName + "\n"
        reply = ""
        message = message.encode('utf-8')    #Encode message
        time.sleep(1)     #give arduino time to set up (there are delays in arduino code for pressure sensor)
        while(1):
            time.sleep(0.5)     #delay before sending message again
            self.ser.write(message)
            # print("Handshake: ", message)
            if self.ser.in_waiting > 0:
                reply = self.ser.readline().strip()
                self.ser.reset_input_buffer()
                reply = reply.decode('ascii')

                if reply == self.pumpName:
                    self.ser.reset_output_buffer()
                    break

        # return open serial connection to allow pumps to be controlled in main code
        return reply



    def sendStep(self, stepNumber):
        """
        This function sends ideal position (stepNumber) then receives
        the real step count (stepCount) from arduino.
        steps = sendStep(serialConnection, stepNumber)
        """
        if type(stepNumber) != str:
            stepString = "{:04d}".format(stepNumber)
        else:
            stepString = stepNumber
        message = "S" + stepString + "\n"
        # print("Message: ", repr(message))
        message = message.encode('utf-8')
        self.ser.write(message)
        return



    def listenReply(self):
        x = "e"
        stepPress = b""
        noBytes = self.ser.in_waiting
        # Wait here for reply - source of delay
        while noBytes == 0:
            noBytes = self.ser.in_waiting
        # Read all bytes in input buffer
        # stepPress = ser.read(noBytes)
        # Check for end character
        while ord(x) != ord("E"):
            x = self.ser.read()
            if x == b"":
                break
            elif x == b"E":
                break
            stepPress = stepPress + x

        stepPress = stepPress.decode('utf-8')
        stepPress = stepPress.split(',')
        # print(stepPress)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        if stepPress == ['']:
            stepCount = "S_Empty" # Change this later to handle dropped values
            pumpPress = "P_Empty"
            pumpTime = "T_Empty"
        else:
            stepCount = stepPress[0]
            pumpPress = float(stepPress[1])/10
            pumpTime = stepPress[2]
        return stepCount, pumpPress, pumpTime


    def listenZero(self, isPumpZero):
        x = "e"
        stepPress = b""
        while self.ser.in_waiting == 0:
            pass
        # Check for end character
        while ord(x) != ord("E"):
            x = self.ser.read()
            if x == b"":
                break
            elif x == b"E":
                break
            stepPress = stepPress + x

        # self.ser.reset_input_buffer()
        # self.ser.reset_output_buffer()
        if stepPress == b"":
            stepCount = "S_Empty" # Change this later to handle dropped values
            pumpPress = "P_Empty"
            pumpTime = "T_Empty"
        else:
            # print(stepPress)
            stepPress = stepPress.decode('utf-8')
            stepPress = stepPress.split(',')
            stepCount = stepPress[0]
            pumpPress = float(stepPress[1])/10
            pumpTime = stepPress[2]

        if (isPumpZero == True):
            stepCount = 0
        return stepCount, pumpPress, pumpTime