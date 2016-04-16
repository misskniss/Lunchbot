import serial
import time

arduino = serial.Serial('COM4', 9600, timeout=.1)

while True:
  time.sleep(1)
  data = arduino.readline()
  if data:
    print "The Arduino sends: " + data.rstrip("\n") 
    raw_input("Press Enter to switch the Arduino's state.")
    arduino.write("a")