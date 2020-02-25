import serial 
import time

testString = "TOP\n"
reply =""
replyLetters = []

top = serial.Serial()

if top.is_open==False:
    top.port = 'COM3'
    top.baudrate = 115200
    top.bytesize = serial.EIGHTBITS
    top.parity = serial.PARITY_NONE
    top.stopbits = serial.STOPBITS_ONE
    top.timeout = 0
    top.open()

time.sleep(2)   #give arduino time to set up
testString = testString.encode('utf-8')    #Encode message
top.write(testString)                      #Send message 
while reply != "TOP":
    while top.in_waiting > 0:
        reply = top.readline().strip()
        reply = reply.decode('ascii')
        print(reply)
print("Escaped")
while True:
    while top.in_waiting > 0:
        reply = top.readline().strip()
        reply = reply.decode('ascii')
        print(reply)

top.close()
