#!/usr/bin/env python2.7

import time
from threading import Thread
 
import sys
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cfclient.utils.logconfigreader import LogVariable, LogConfig

magn_ellipsoid_center = [1287.10, -501.436, 44.8821]
magn_ellipsoid_transform = [[0.919091, 0.0191566, 0.0206570], [0.0191566, 0.934831, -0.00394435], [0.0206570, -0.00394435, 0.994711]]

# ellipse fitting
import numpy as np
from numpy.linalg import eig, inv

def fit_ellipse(x, y):
    x = x[:,np.newaxis]
    y = y[:,np.newaxis]
    D =  np.hstack((x*x, x*y, y*y, x, y, np.ones_like(x)))
    S = np.dot(D.T,D)
    C = np.zeros([6,6])
    C[0,2] = C[2,0] = 2; C[1,1] = -1
    E, V =  eig(np.dot(inv(S), C))
    n = np.argmax(np.abs(E))
    a = V[:,n]
    return a

def ellipse_center(a):
    b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
    num = b*b-a*c
    x0=(c*d-b*f)/num
    y0=(a*f-b*d)/num
    return np.array([x0,y0])

def fit_ellipse_center(xs, ys):
    a = fit_ellipse(np.array(xs), np.array(ys))
    c = ellipse_center(a)
    return c[1], c[0] # looks like I need to swap it

class Main:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yawrate = 0
        self.thrust = 0
        self._stop = 0;
        
        self.series_index = -1
        self.series_data = []
        self.series_thrust = [0, 10001, 20001, 30001, 40001, 50001, 60000]
        self.discard_data = True

        self.crazyflie = Crazyflie()
        cflib.crtp.init_drivers()
 
        # You may need to update this value if your Crazyradio uses a different frequency.
        self.crazyflie.open_link("radio://0/10/250K")
        self.crazyflie.connectSetupFinished.add_callback(self.connectSetupFinished)
 
    def connectSetupFinished(self, linkURI):
        print "Should be connected now...\n"

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
        self.thrust = 0

        # make sure we actually set the thrust to zero before we kill the thread that sets it
        time.sleep(0.5) 
        self._stop = 1;

        self.process_data()
        
        # Exit the main loop
        print "Exiting main loop in 1 second"
        time.sleep(1)
        self.crazyflie.close_link() # This errors out for some reason. Bad libusb?
        sys.exit(0)

    def process_data(self):
        centers = []
        for series in self.series_data:
            xs = []
            ys = []
            zs = []
            for x, y, z in series:
                # correct hard and soft iron errors
                x = x - magn_ellipsoid_center[0]
                y = y - magn_ellipsoid_center[1]
                z = z - magn_ellipsoid_center[2]
                prod = np.dot(magn_ellipsoid_transform, [[x], [y], [z]]) 
                x = prod[0][0]
                y = prod[1][0]
                z = prod[2][0]
                # store corrected values
                xs.append(x)
                ys.append(y)
                zs.append(z)
            center_x, center_y = fit_ellipse_center(xs, ys)
            center_z = np.median(np.array(zs))
            centers.append((center_x, center_y, center_z))
        self.curve_fit(centers)
        #self.show_plot()

    def show_plot(self):
        from mpl_toolkits.mplot3d import Axes3D
        import matplotlib.pyplot as plt

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')

        for series in self.series_data:
            xs = []; ys = []; zs = []
            for x, y, z in series:
                xs.append(x)
                ys.append(y)
                zs.append(z)
            ax.scatter(np.array(xs), np.array(ys), np.array(zs))
        plt.show()

    def curve_fit(self, centers):
        nn = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        xs = []
        ys = []
        zs = []
        x0, y0, z0 = centers[0] 
        for i in range(0, 7):
            x, y, z = centers[i]
            xs.append(x0 - x)
            ys.append(y0 - y)
            zs.append(z0 - z)
        print 'result'
        print 'qx =', list(np.polyfit(nn, xs, 3))
        print 'qy =', list(np.polyfit(nn, ys, 3))
        print 'qz =', list(np.polyfit(nn, zs, 3))
 
    def input_loop(self):
        print "Beginning input loop:"
        while True:
            command = raw_input("Press Enter for next iteration (e or q will quit):")
            if (command=="exit") or (command=="e") or (command=="quit") or (command=="q"):
                self.stop()
            elif command == '':
                if self.series_index < 6:
                    self.discard_data = True
                    time.sleep(0.5)
                    # do stuff
                    self.series_index = self.series_index + 1
                    print 'Running next round, press rotate CF and hit enter when finished. Iteration', self.series_index
                    self.series_data.append([])
                    self.thrust = self.series_thrust[self.series_index]
                    time.sleep(0.5)
                    self.discard_data = False
                else:
                    print 'Finished calibration'
                    self.stop()
    
    def pulse_command(self):
        while 1:
            self.crazyflie.commander.send_setpoint(self.roll, self.pitch, self.yawrate, self.thrust)
            time.sleep(0.1)
     
            # Exit if the parent told us to
            if self._stop==1:
                return
 
    def magData(self, data):
        x, y, z = data['mag.x'], data['mag.y'], data['mag.z']
        if not self.discard_data:
            self.series_data[self.series_index].append((x, y, z))
 
Main()
