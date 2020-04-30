"""
Set of functions to facilitate connection to syringe pumps, which are controlled by arduinos.
Arduinos connected to a USB hub so COM port names must be checked in main code.
Enables connection, pressure monitoring, and step count monitoring for later burst
protection and volume calculation.
"""
import serial 
import time

def connect(pumpName, portNumber):
    """
    Each pump requires verification of the cable it is contolling (top, lhs, or rhs).
    Pumps will function normally when the pumpName sent here matches the name hardcoded 
    on each arduino.
    e.g. top = connect("TOP", 4)
    """
    message = pumpName + "\n"
    reply = ""
    port = 'COM%s' % (portNumber)

    #Open serial port at given COM port at 115200 baud rate
    ser = serial.Serial(port, 115200)
    # time.sleep(1)   
    message = message.encode('utf-8')    #Encode message
    while(1):
        ser.write(message)
        time.sleep(1)     #give arduino time to set up (there are delays in arduino code for pressure sensor)
        if ser.in_waiting > 0:
            reply = ser.readline().strip()
            ser.reset_input_buffer()
            reply = reply.decode('ascii')
            if reply == pumpName:
                ser.reset_output_buffer()
                break
    # return open serial connection to allow pumps to be controlled in main code
    return ser, reply



def listenStepPress(ser, stepNumber):
    """
    This function sends ideal position (stepNumber) then receives
    the real step count (stepCount) from arduino.
    steps = sendStep(serialConnection, stepNumber)
    """
    message = "S" + str(stepNumber) + "\n"
    # print("Message: ", message)
    message = message.encode('utf-8')
    ser.write(message)
    stepPress = ser.readline().strip()
    stepPress = stepPress.decode('ascii')
    # print("Response: ", stepPress)
    stepPress = stepPress.split(',')
    ser.reset_input_buffer()
    if stepNumber != "Closed":
        stepCount = stepPress[0]
        pumpPress = stepPress[1]
        pumpTime = stepPress[2]
    else:
        stepCount = stepPress[0]
        pumpPress = 0
        pumpTime = stepPress[1]
    return stepCount, pumpPress, pumpTime
