def connect(pumpName, portNumber) :
    import serial 
    import time

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
        while ser.in_waiting > 0:
            reply = ser.readline().strip()
            reply = reply.decode('ascii')
            print(reply)

    #ser.close()
    return ser

def listen(ser):
    return
