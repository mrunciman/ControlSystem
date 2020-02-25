# Set of functions to facilitate connection to syringe pumps, which are controlled by arduinos.
# Arduinos connected to a USB hub so COM port names must be checked in main code.
# Enables connection, pressure monitoring, and step count monitoring for later burst
# protection and volume calculation.
import serial 
import time



### connect() ###
# Each pump requires verification of the cable it is contolling (top, lhs, or rhs).
# Pumps will function normally when the pumpName sent here matches the name hardcoded 
# on each arduino.
def connect(pumpName, portNumber) :

    message = pumpName + "\n"
    reply = ""
    port = 'COM%s' % (portNumber)

    #Open serial port at given COM port at 115200 baud rate
    ser = serial.Serial(port, 115200)
    # print(ser)

    time.sleep(1)   #give arduino time to set up (there are delays in arduino code for pressure sensor)
    message = message.encode('utf-8')    #Encode message
    ser.write(message)                   #Send message 
    while reply != pumpName:
        if ser.in_waiting > 0:
            reply = ser.readline().strip()
            reply = reply.decode('ascii')
            ser.reset_input_buffer()
            print(reply)

    # return open serial connection to allow pumps to be controlled in main code
    return ser



### listenSteps() ###
# This function receives the step count sent from the arduino
def listenSteps(ser):
    message = "S"
    message = message.encode('utf-8')    #Encode message
    ser.write(message)                   #Send message
    steps = ser.readline().strip()
    steps = steps.decode('ascii')
    ser.reset_input_buffer()
    #print(steps)
    return steps



### listenPress() ###
# This function receives the pressure value sent from the arduino
def listenPress(ser):
    message = "P"
    message = message.encode('utf-8')    #Encode message
    ser.write(message)                   #Send message
    pressure = ser.readline().strip()
    pressure = pressure.decode('ascii')
    ser.reset_input_buffer()
    #print(pressure)
    return pressure