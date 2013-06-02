#!/usr/bin/env python2.7

import time
from threading import Thread
 
import sys
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cfclient.utils.logconfigreader import LogVariable, LogConfig

class Main:
    def __init__(self):
        self._stop = 0
        self.crazyflie = Crazyflie()
        cflib.crtp.init_drivers()

        # You may need to update this value if your Crazyradio uses a different frequency.
        self.crazyflie.open_link("radio://0/10/250K")
        self.crazyflie.connectSetupFinished.add_callback(self.connectSetupFinished)
 
    def connectSetupFinished(self, linkURI):
        lg = LogConfig("Magnetometer", 100)
        lg.addVariable(LogVariable("mag.x", "int16_t"))
        lg.addVariable(LogVariable("mag.y", "int16_t"))
        lg.addVariable(LogVariable("mag.z", "int16_t"))
            
        log = self.crazyflie.log.create_log_packet(lg)
        if log is not None:
            log.dataReceived.add_callback(self.magData)
            log.start()
        else:
            print "Magnetometer not found in log TOC"        
        
        # Keep the commands alive so the firmware kill-switch doesn't kick in.
        Thread(target=self.pulse_command).start()
        Thread(target=self.input_loop).start()

    def stop(self):
        # Exit command received
        # Set thrust to zero to make sure the motors turn off NOW

        # make sure we actually set the thrust to zero before we kill the thread that sets it
        time.sleep(0.5) 
        self._stop = 1;

        # Exit the main loop
        time.sleep(1)
        self.crazyflie.close_link() # This errors out for some reason. Bad libusb?
        sys.exit(0)

    def input_loop(self):
        command = raw_input("")
        self.stop()
    
    def pulse_command(self):
        while 1:
            self.crazyflie.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.1)
     
            # Exit if the parent told us to
            if self._stop==1:
                return
 
    def magData(self, data):
        x, y, z = data['mag.x'], data['mag.y'], data['mag.z']
        try:
            print x, y, z
            sys.stdout.flush()
        except:
            self.stop()
 
Main()
