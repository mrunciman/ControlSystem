"""
Set of functions to facilitate connection to syringe pumps, which are controlled by arduinos.
Arduinos connected to a USB hub so COM port names must be checked in main code.
Enables connection, pressure monitoring, and step count monitoring for later burst
protection and volume calculation.
"""
import serial 
import time

#Arduino clock frequency
CLOCK_FREQ = 16000000
# Arduino prescalar value
PRESCALER = 8


def connect(pumpName, portNumber) :
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
    # print(ser)
    # print(pumpName)
    time.sleep(2)   #give arduino time to set up (there are delays in arduino code for pressure sensor)
    message = message.encode('utf-8')    #Encode message
    while reply != pumpName:
        ser.write(message)        #Send message 
        if ser.in_waiting > 0:
            reply = ser.readline().strip()
            reply = reply.decode('ascii')
            ser.reset_input_buffer()

    # return open serial connection to allow pumps to be controlled in main code
    return ser, reply



def sendStep(ser, stepNumber):
    """
    This function sends ideal position (stepNumber) then receives
    the real step count (stepCount) from arduino.
    steps = sendStep(serialConnection, stepNumber)
    """
    message = "S" + str(stepNumber) + "\n"
    message = message.encode('utf-8')
    ser.write(message)
    if stepNumber != "Closed":
        stepCount = ser.readline().strip()
        stepCount = stepCount.decode('ascii')
        ser.reset_input_buffer()
        return stepCount
    else:
        return stepNumber



def sendOCR(ser, OCR):
    """
    This function sends the desired step frequency 
    to the serial port ser. Negative and positve values acccepted
    and dealt with on arduino.
    """
    # Add newline character on end for ease of reading on arduino
    message = "O"
    if OCR < 0:
        message = message + str(OCR) + "\n"
    else:
        message = message + "+" + str(OCR) + "\n"
    # print(message)
    message = message.encode('utf-8')    #Encode message
    ser.write(message)                   #Send message
    reply = ser.readline().strip()
    reply = reply.decode('ascii')
    ser.reset_input_buffer()
    return reply




def listenPress(ser):
    """
    This function asks for then receives the pressure read by
    the arduino over serial.
    pressure = listenPress(serialConnection)
    """
    message = "P"
    message = message.encode('utf-8')    #Encode message
    ser.write(message)                   #Send message
    pressure = ser.readline().strip()
    pressure = pressure.decode('ascii')
    ser.reset_input_buffer()
    #print(pressure)
    return pressure



def sendFreq(ser, freq):
    """
    This function sends the desired step frequency 
    to the serial port ser. Negative and positve values acccepted
    and dealt with on arduino.
    """
    # Add newline character on end for ease of reading on arduino
    message = "F"
    if freq != 0:
        OCR = CLOCK_FREQ/(PRESCALER*freq)-1
        message = message + str(OCR)
    else:
        OCR = 0
    message = message.encode('utf-8')    #Encode message
    ser.write(message)                   #Send message
    reply = ser.readline().strip()
    reply = reply.decode('ascii')
    ser.reset_input_buffer()
    return OCR