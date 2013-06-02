Bitcraze Crazyflie magnetometer calibration scripts
===================================================

This repo contains scripts for magnetometer calibration of Bitcraze Crazyflie 10DOF http://www.bitcraze.se/ quadcopter.

One of the scripts uses code from https://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs

Please perform all calibration away from the objects which can casue local magnetic field distortions (metal objects, live wires, etc).

# Hard Iron and Soft Iron Calibration

The raw data retuned by CF magnetometer are affected by various distorions produced by CF components. To compensate
these distortions magnetometer must be calibrated. To peform calibration, carry out following steps:

1. Clone, build and flash CF firmware from: https://bitbucket.org/ak1394/crazyflie-firmware 

2. Download and install Processing environment from http://www.processing.org/

3. Install EJML library into processing environment. Follow this guide to install it http://wiki.processing.org/w/How_to_Install_a_Contributed_Library When downloading EJML jars use version 0.21 available at this link https://code.google.com/p/efficient-java-matrix-library/downloads/detail?name=ejml-0.21.jar as newer versions of EJML compiled with Java 1.7 while Processing uses Java 1.6. At this point you should be able to run Magnetometer_calibration Processing script, which should display white sphere in the centre of black screen.

4. Verify that you can run calibrate_hardsoft.py script (make sure you have correct PYTHONPATH set for loading CF libraries). Turn on CF, run calibrate_hardsoft.py and observe its output. You should see lines being printed on the screen, whith each line containing three numbers. Rotating CF will change the numbers being printed. Hitting Enter will stop the script.

5. The Magnetometer_calibration script is designed to read sampling data over the network, which allows to run calibrate_hardsoft.py on CF Virtual machine, and run Processing on the host. Edit 'setup' function in Magnetometer_calibration and replace the IP address with the IP address of VM. If you're running both Magnetometer_calibration and calibrate_hardsoft.py on the same host, use 127.0.0.1 IP address.

6. Start calibrate_hardsoft.py redirecting it's output to the network (using netcat):

    python calibrate_hardsoft.py | nc -l 1234

7. Start Magnetometer_calibration script and rotate CF over multiple axis observing Magnetometer_calibration screen. As you rotate CF you should see more dots appearing on the screen. After a while you should see something similar to mag_cal.png. Hit Enter and inspect Processing console output, you should see output similar to the output below:

    magn_ellipsoid_center = [1331.69, -553.710, 15.8198]
    magn_ellipsoid_transform = [[0.984237, 0.0158896, -0.00133159], [0.0158896, 0.980074, -0.00996596], [-0.00133159, -0.00996596, 0.967180]]

Copy the output to a text editor. These are the values you will need to insert into calibrate_powered.py script and also into the lib/cfclient/ui/tabs/FlightTab.py of CF PC Client.

# Hard Iron calibration for engine on mode

During the flight, higher currents on the CF cause additional hard iron distortions. To correct these, we sample CF magnetometer output at six various speeds motor speeds.

Because standard CF firmware tries to adjust motor output as CF rotates, we must use patched firmware that ignores CF gyro/accelerometer output and allows to keep same motor speed regardless of CF orientation. Since the calibration is performed with motors on, you'll need to strap down CF so it wont fly away.

1. Using the same modified firmware sources from step 1 of previous section, copy stabilizer-calibrate-powered.c to modules/src/stabilizer.c (make backup of the original stabilizer.c or make sure you know how to restore it), make and flash firmare to CF.

2. Start calibrate_powered.py script. You should see propmt asking "Press enter for next iteration...." at this point start rotating strapped-down CF around vertical axis, making flat 360 degree turn and returning it to its original position, then hit Enter and repeat rotation. You need to make 7 rotations in total. The first rotation is performed with engines off, each subsequent rotation increases motor power. Once you completed all rotations, CF engines will shut down and similar output will be displayed on the screen (copy it and save to your editor):

        qx = [0.067946222436498283, -0.25034004667098259, 8.3336994198409666, -0.17762637163222378]
        qy = [-0.13945102271766135, 2.9074808469097495, 1.6764850422889934, 0.19244505046927501]
        qz = [0.018800599305554239, -0.79590273035713055, -3.1033531112103478, 0.13550993988096199]

3. Revert changes to firmware sources you performed in step 1, build and flash firmware to CF.

# Running modified CF PC Client

1. Clone sources from: https://bitbucket.org/ak1394/crazyflie-pc-client

2. Open lib/cfclient/ui/tabs/FlightTab.py file and replace lines around line 185 with values generated in previous steps.

3. Run client and connect to CF. You should see compass widget displayed in Flight Control tab. As you rotate CF the compass indication should reflect CF movement.

4. The heading lock is hardcoded in the sources, and set to 0 degrees north. If you want to adjust it or turn it off look in lib/cfclient/ui/tabs/FlightTab.py around line 235

5. Be aware that the local magnetic fields indoors, can change quite significantly. In my case compass works quite well in one of the rooms, but doesn't work at all in the differen one.

Happy Flying!
