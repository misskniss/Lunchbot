#! /bin/bash/python

# Author:   Jen Kniss
# Date:     Spring 2016
# Purpose:  BSU Turtlebot Research, ECE 474
#           Interface over serial conection with the scooper.ino code
#            from Robert

import serial
import time
import serial.tools.list_ports


"""
    This class reads states and sends state changing signals
    to the arduino controlling the scoop on TurtleBot.

    The states are as follows for the scoop:
    State 1: Scoop open and listening to the sonic sesnor for an egg
             TurtleBot waits to hear state 2 signal to indicate "Stop moving"
    State 2: Scoop has detected an egg and started to close
             Turtlebot stops moving and waits for State 3 signal
    State 3: Scoop is fully closed, egg in custody!
             Turtlebot drives "home" and sends state change signal when home
             to indicate "Drop the egg"
    State 4: Scoop opens and ignores the sonic sensor to allow TurtleBot to 
             get clear of the egg without triggering a scoop close
             Turtlebot sends change state signal when ready to enter State 1 
"""
class SerialTest():

    """ Bring up Serial communication link between Turtlebot
        and Scoop (Arduino)
    """
    def __init__(self):
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            name = p[0]
            if ("ACM" in name):
                print "Connecting to serial port: " + name
                port = name
        try: 
            self.link = serial.Serial(port, 9600)
            self.link.setDTR(False)
            time.sleep(0.022)
            self.link.setDTR(True)
        except:
            self.link = serial.Serial(port, 9600)
            print "Ignoring failed attempt to reset the Serial device. "
        self.link.isOpen()
        time.sleep(1)
        self.data = None
        self.state = 0
        print "SerialTest: Init complete."


    """ Read state and strip extra characters
        @return state
    """
    def read(self):
        try:
            self.data = self.link.readline()
            time.sleep(1)
        except:
            self.state = "3"
        if self.data:
            self.state = self.data.strip("\r\r\n")
            time.sleep(1)
            print "SerialTest.read(): Got state: " + self.state
        return self.state
    
    """ Send state change signal "*"
    """
    def change_state(self):
        print "SerialTest.change_state()"
        self.link.write("*")


if __name__ == '__main__':
    test = SerialTest()
    
    state = test.read()
    print type(state)
    print "Start state is: " + str(state)
    while(state != None):
        if state is "4":
            print "State is 4"
            print "Changing state to 1..."
            test.change_state()
            state = test.read()
        if state is "1":
            print "State is 1"
            print "Waiting for egg..."
            state = test.read()
        if state is "2":
            print "State is 2. "
            print "Scoop is closing..."
            state = test.read()
        if state is "3":
            print "State is 3"
            print "Changing state to 4..."
            test.change_state()
            state = test.read()
