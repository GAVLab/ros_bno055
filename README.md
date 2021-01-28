# ros_bno055

A ROS node for communicating with the [bno055](https://www.adafruit.com/product/4646) 9DOF IMU.

## Description

The node communicates with the BNO08x via i2c on the Raspberry Pi using Adafruit's Python library: https://github.com/adafruit/Adafruit_CircuitPython_BNO055.  The i2c address is preset to `0x28`.  The data is stored in the following:

* Accelerometer and Gyroscope (sensor_msgs/Imu): `bno055/raw`
* Magnometer (sensor_msgs/MagneticField): `bno055/mag`
* Temperature (sensor_msgs/Temperature): `bno055/temp`
* Diagnostics (diagnostic_msgs/DiagnosticStatus): `bno055/status`

## Installation Instructions

* Enable i2c
  ```
  sudo apt-get install i2c-tools
  i2cdetect -l
  ```
* Add `i2c-devl` to boot with `sudo nano /etc/modules-load.d/modules.conf`
* Install wiringpi `sudo apt install wiringpi`
* Connect i2c devices to Sparkfun Qwiic hat and run `i2cdetect -y 1` to identify channels
* Install Circuit Python: https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi
* Install driver for bno055 IMU: `sudo pip3 install adafruit-circuitpython-bno055`.

## Running the Node

* Option 1: `rosrun ros_bno055 talker.py` 
* Option 2: `roslaunch launches/bno055.launch`
  
## Tested Setup

It should work on other versions but Python 3 is a requirement.

* Platform: Raspberry Pi 4
* OS: Ubuntu MATE 20.04
* ROS: Noetic
* Python: 3.8.5

