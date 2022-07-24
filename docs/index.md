# Diy Arduino Gimbal

A 3D printed do-it-yourself (DIY) gimbal - fitted with 3 FS90 Micro servo motors - is controlled using an MPU6050 inertial measurement unit (IMU) sensor in order to keep the 3D printed platform stable during rotation in any axis. The servo motors and the IMU sensor are connected to the Arduino Nano microcontroller.

(To be completed)

## Hardware

### Components

The following components (and tools) were used in this project:

- 3D printed gimbal parts

- Arduino Nano (and usb cable)

- MPU6050 accel/gyro module

- 3 x FS90 Micro servo motors

- LM2596 Buck Regulator

- 9V battery and connector

- 2.54 mm female and male headers

- Loose cables to solder on female/male headers

- Veroboard to solder on female headers (for the arduino nano and MPU6050) and
male headers for the servo cables

- On/Off switch

- Soldering iron

- Screwdriver set

- Double sided tape

- Duck tape

### Project Wiring

The diagram below shows how the electronic components of the gimbal are connected.

<p float="center">
  <img src=images/gimbal_wiring_diagram.png width="600">
</p>

The 9V battery provides the power for the project. It is connected to a buck/voltage regulator which steps down the voltage to 5V for the Arduino Nano. A switch is connected inline with the regulator and battery to turn on/off the circuit. 

The MPU6050 board pins were connected to the pins of the Nano as follows:

| MPU6050 board | Arduino Nano board |
| ----------- | ------------------ |
| VCC         | 5V |
| GND         | GND |
| SCL         | A5 |
| SDA         | A4 |
| INT         | D2 |

The pair of analog pins, A4 and A5, on the Nano are usually used for I2C communications according to the board pinout schematics. Hence, pins A4 and A5 are connected to SDA and SCL on the MPU6050 module respectively. The external interrupt pin of the MPU6050 is connected to the D2 digital Nano pin.

The signal cable of the servo motors are connected to the Nano as follows:

| Servo | Arduino Nano board |
| ----- | ------------------ |
| Roll  | D4 |
| Pitch | D5 |
| Yaw   | D10 |


### Assembly
The physical parts of the gimbal: the platform, servo housings and the main body were 3D printed. The STL files for the project can be found [here](https://drive.google.com/file/d/1nKLQ7iNL-5FeuZThrUJ0UgsAtbLwWR5H/view?usp=sharing) from Make It Smart. 

<p float="center">
  <img src=images/components.png width="600">
</p>

The Nano and MPU6050 boards were attached to a breadboard using female headers. The servo motors were connected to the Nano via male headers soldered onto the breadboard (to the left of the Nano in the image below). The signal, ground and vcc connections were soldered underneath the breadboard to the corresponding pins on the Nano. 

An extra row of headers was attached for debugging power and ground connections. 

<p float="center">
  <img src=images/boards_closeup.png width="600">
</p>

The black and red wires shown are connected to the ground and the voltage output from the buck regulator. The idea behind this connection and assembly is again attributed to [Make It Smart](https://www.youtube.com/watch?v=1aRJvid5Ib4). The servos are attached to the servo housings with screws, the double sided tape is used to attach the breadboard securely in the gimbal body, and the duck tape is used to insulate connections from possible short circuits. 

The fully assembled gimbal is shown in the pictures below.

<p float="center">
  <img src=images/assembled_front.png width="600">
  <img src=images/assembled_side.png width="600">
  <img src=images/assembled_top.png width="600">
</p>

## Software and libraries used
The [Arduino IDE](https://www.arduino.cc/en/software) is used in programming this project. The [i2cdevlib](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev) and [MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050) libraries by Jeff Rowberg were also used herein. These libraries can be installed by importing them into the Arduino IDE as explained in the [README](https://github.com/jrowberg/i2cdevlib) of the repository; this [guide](https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries) can also be instructional for the installation process.

## I2C


## MPU6050


### MPU6050 Calibrations


### Sketch Upload

<p float="center">
  <img src=images/sketch_upload.png width="600">
</p>

## Working principle

## Video demonstration

## Observations

## Future work/suggestions
- Smaller voltage regulator
- Use a smaller battery
- connect directly to Vin

## References
Libraries:

Additional information on i2cdevlib https://www.i2cdevlib.com