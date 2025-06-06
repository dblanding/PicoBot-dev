# Inspiration for improving the PicoBot
* The book [**Robotics at Home w/ Pico**](/home/doug/Downloads/Robotics_at_home_with_Pico.pdf) by Danny Staples demonstrates (in CircuitPython) some useful ideas that can be introduced to my PicoBot
    * Using BLE UART Friend modules to facilitate teleop controls and laptop data collection & display
    * Using the Bosche BNO055 IMU connected via I2C
        * instead of the BNO08x module in RVC mode (on uart)
            * which needed to be running at 10x loop speed in order to be responsive
    * I ended up learning how to connect multiple VCSEL distance sensors using an I2C multiplexer board.
        * Can have 3 of them giving left/right/fwd dist measurements
* This book also introduced me to the possibility of using Monte Carlo Localization (MCL) and pointed me to some related resources
    * [Video lectures](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQEn33QDVGJpiZLi-SlL7vA) by Cyrill Stachniss (which are based on the book *Probabilistic Robotics*)
    * My latest thinking is to send pose and distance sensor data to the laptop and have a program on the laptop do the processing (Localization, Mapping, etc)
    * At some point, I may explore using the RPLidar A1 to collect and send 360-degree distance data.

## The new, improved PicoBot

* Tele-operate the PicoBot using a joystick type control that senses tipping/tilting of a cell phone
    * This works by sending the phone's accelerometer data to the robot using BLE
* An additional Bluefruit LE UART Friend device allows the robot to send map and pose data to my laptop.
    * For this, I need to free up a 2nd uart channel
        * One uart channel is for sending tele-op driving commands from my phone
        * The second is for sending pose data and the arena map to a program on my laptop which plots the data.
* Up until now, the Picobot has used a bno08x IMU (in RVC mode) on uart1
    * In order to free up this uart, I will instead use a bno055 IMU connected on one of the I2C buses
    * This [video tutorial](https://core-electronics.com.au/videos/raspberry-pi-pico-workshop-chapter-44-i2c) shows how to do this.
    * I end up with one VCSEL distance sensor on i2c0 and the other sharing the i2c1 bus with the IMU.

### Running the code:
* [laptop folder](laptop) contains files that are run on the laptop
* [robot folder](robot) contains files that are run on the Pico

* to Run:
    1. Place the PicoBot at pose (0, 0, 0) on the floor of the arena.
    2. Turn the PicoBot power switch to **ON**. This starts the file `main.py` on the robot.
    3. Open the Bluefruit connect app (cell phone)
        * then connect to the first listed of the 2 BLE UART Friend devices
        * then select Controller
        * then select Accelerometer.
    4. On the laptop, run the file `display_from_robot.py`. It's important to have the phone app already connected so the laptop has no other choice but to connect to the device on uart1.

    ![bluefruit connect app](imgs/bluefruit_connect_app.png)

### [Here](early_code.md) are some notes on some of my early attempts at getting the BLE communication working.

### Revised circuitboard configuration
* With the addition of 2 BLE UART Friend modules and multiple sensors sending data on I2C, an I2C multiplexer has also been added.
* A full size breadboard is needed to hold everything.
* Took the opportunity to put the right and left VCSEL distance sensors directly above the wheels.

![PicoBot electronics](imgs/picobot_electronics.jpg)
* Added forward looking VCSEL distance sensor

![PicoBot w/ all 3 distance sensors](imgs/with_fwd_dist_sensor.jpg)

* Use vector addition to calculate the location of detected points.

![Vector Addition](imgs/vector-addition.jpg)

### Customize arena and plot points detected by distance sensors
* Starting at origin of arena with pose = (0, 0, 0)

![Starting at Home](imgs/run1_start.png)

* Run No. 1 
    * Begin driving (in Tele-Op mode) CCW around arena perimeter
    * plotting only points detected by right distance sensor (in red)

![Run #1](imgs/run1a.png)

* Run No. 2 plotting distance values (+ offset) from all 3 sensors 
    * Left -> RED
    * Right -> GREEN
    * Front -> YELLOW

![Run #2](imgs/run2.png)

### Revise driving program: Replace Tele-Operation with a program to drive along a Search & Rescue pattern
* First, do a simple right turn and adjust TRACK_WIDTH until Odometer pose_angle agrees with IMU yaw angle
* Next, drive a long straight leg with a goal_angle = 0 to make sure that pose_angle and yaw are nearly equal at the end of the leg.
* Now we're ready to drive the S&R pattern
    * Rotate arena axis 90 degrees from before
    * Use Start button to start driving
    * The Stop button to stop driving
    * Initially, robot is at pose (0, 0, 0) (aimed to the right)
    * Make an initial 90 deg turn to the right, then
    * Sync pose_angle to yaw just prior to driving in the -Y direction
    * Steer to a target angle
        * using yaw error as Proportional feedback
        * gyro-z as Derivative feedback

![S&R pattern](imgs/s&r.png)

* Trip [data](data/s&r_data.txt) shows the record of the trip

### Implement selectable Tele-Op or Autonomous Driving

![Autonomous](imgs/sr.png)

![Tele-Op](imgs/to.png)

