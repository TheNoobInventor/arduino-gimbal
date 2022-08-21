# Diy Arduino Gimbal

A 3D printed do-it-yourself (DIY) gimbal - fitted with 3 FS90 Micro servo motors - is controlled using an MPU6050 inertial measurement unit (IMU) sensor to keep the 3D printed platform stable during rotation in any axis. The servo motors and the IMU sensor are connected to the Arduino Nano microcontroller.

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
  <img title='Wiring diagram' src=images/gimbal_wiring_diagram.png width="600">
</p>

The 9V battery provides the power for the project and is connected to a buck/voltage regulator which steps down the voltage to 5V for the Arduino Nano. A switch is connected inline with the regulator and battery to turn on/off the circuit. 

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
The physical parts of the gimbal: the platform, servo housings and the main body were 3D printed. The STL files for the project can be found [here](https://drive.google.com/file/d/1nKLQ7iNL-5FeuZThrUJ0UgsAtbLwWR5H/view?usp=sharing) from Make It Smart and are also provided in the project repository. 

<p float="center">
  <img title='Components' src=images/components.png width="600">
</p>

The Nano and MPU6050 boards were attached to a breadboard using female headers. The servo motors were connected to the Nano via male headers soldered onto the breadboard (to the left of the Nano in the image below). The signal, ground and vcc connections were soldered underneath the breadboard to the corresponding pins on the Nano. 

An extra row of headers was attached for debugging power and ground connections. 

<p float="center">
  <img title='Boards closeup' src=images/boards_closeup.png width="600">
</p>

The black and red wires shown are connected to the ground and the voltage output from the buck regulator. The idea behind this connection and assembly is again attributed to [Make It Smart](https://www.youtube.com/watch?v=1aRJvid5Ib4). The servos are attached to the servo housings with screws, the double sided tape is used to attach the breadboard securely in the gimbal body, and the duck tape is used to insulate connections from possible short circuits. 

The fully assembled gimbal is shown in the pictures below.

<p float="center">
  <img title='Assembled front' src=images/assembled_front.png width="600">
  <img title='Assembled side' src=images/assembled_side.png width="600">
  <img title='Assembled top' src=images/assembled_top.png width="600">
</p>

## Software and libraries
The [Arduino IDE](https://www.arduino.cc/en/software) is used in programming this project. The [i2cdevlib](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev) and [MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050) libraries by Jeff Rowberg were also used herein. These libraries can be installed by importing them into the Arduino IDE as explained in the [README](https://github.com/jrowberg/i2cdevlib) of the repository; this [guide](https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries) can also be instructional for the installation process. 

## MPU6050 module
The MPU6050 module, shown below, is a Micro Electro-Mechanical System (MEMS) which consists of a 3-axis gyroscope -- to measure angular velocity -- and a 3-axis accelerometer, to measure acceleration. The module is also called an Inertial Measurement Unit (IMU) which provides a 6-degree of freedom system for motion tracking and measures velocity, orientation, acceleration, displacement and other motion related parameters. 

Some applications of the MPU6050 module are as follows:

- IMU Measurement
- Drones/quadcopters
- Robotic arm controls and
- Self balancing robots

As stated earlier, the MPU6050 module is employed to balance the gimbal platform, how this works will elaborated on in a subsequent section. 

<p float="center">
  <img title='MPU6050' src=images/mpu6050.JPG width="600">
</p>

The pinout of the MPU6050 is shown below

| Pin | Description | 
| --- | ----------- |
| VCC | 3 - 5V
| GND | Ground
| SCL | Serial Clock
| SDA | Serial Data
| XDA | Auxiliary Serial Data
| XCL | Auxiliary Serial Clock
| ADD | I2C address select
| INT | Interrupt

Other features of the module are as follows:

- In-built Digital Motion Processor (DMP) provides high computational power for complex calculations
- In-built 16-bit ADC provides high accuracy
- Uses an Inter-Integrated Circuit (I2C) module to interface with devices like an Arduino using the I2C protocol
- 3-axis gyroscope (angular rate sensor) with a sensitivity up to 131 LSBs/dps and a full-scale range of ±250, ±500, ±1000, and ±2000dps
- 3-axis accelerometer with a programmable full scale range of ±2g, ±4g, ±8g and ±16g
- In-built temperature sensor

Additional features and specifications can be found in the [MPU6050 datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf).

As stated earlier, the I2C protocol is used to interface the MPU6050 module with the Arduino Nano. Jeff Rowberg's library employs the Arduino [Wire library](https://www.arduino.cc/reference/en/language/functions/communication/wire/) used to enable communication between I2C devices. 

### Three Dimensional (3D) Rotations
Before delving further into the project, it's essential to gain some understanding of the fundamentals of rotations in 3D space. 

Firstly, the terms *orientation* and *rotation* have to be defined. Orientation can be defined as the current angular position of an object, while rotation is the operation or action that changes the object's orientation. 

Typically, coordinate frames are used in representing 3D orientation and rotations of an object. This is necessary so that the object has a reference in the real world (in relation to itself) to locate itself, avoid obstacles and more. In a simplified instance, there is a coordinate frame on the object (consider the yellow ball in the figure below), usually known as the body-fixed frame, ***B***, and a reference frame commonly known as the world/earth-fixed frame, ***W***. With two coordinate frames, parameters that are measured or observed in the world-fixed frame can be related in the body-fixed frame and vice versa.

<p float="center">
  <img title='Frames' src=images/coordinate_frames.jpg width="600">
</p>

However, only the body-fixed coordinate frame will be considered in representing the orientations and rotations in this project.

The common methods of representing 3D rotations are:

- Three-angle representation
- Axis angle and
- Quaternions

For brevity, only the three-angle representation and quaternions will be considered here.

#### Three-angle representation
As the name implies, in this method, the orientation and rotation of an object in 3D space is represented using three angles namely: **𝜃** (for rotation around the z-axis), **𝜙** (for rotation around the y-axis), and **𝜓** (for rotation around the x-axis). These angles are known as **Euler angles** with the direction of the axes determined by the [right-hand rule convention](https://en.wikipedia.org/wiki/Right-hand_rule#Coordinates). 

Rotations using the three-angle representation method can be carried out in two ways:
- Euler rotation sequence or
- Cardan rotation sequence. 

In the Euler rotation sequence, rotation occurs about two axes but no successive rotation about the same axis: *XYX*, *YXY*, *YZY*, *ZYZ*, *XZX*, and *ZXZ*.

However, in the Cardan rotation sequence, rotation occurs about all three axes: *XYZ*, *XZY*, *YXZ*, *YZX*, *ZYX*, *ZXY*. Sometimes Cardan rotation sequences are considered a more specific case of Euler rotation sequences -- effectively categorizing all sequences as Euler rotation sequences. Another point to note is that the rotation sequences are non-commutative, meaning sequence *XYZ* ≠ *ZYX*.

<p float="center">
  <img title='Cardan Rotation' src=images/cardan_rotation.png width="600">
</p>

*ZYX Cardan rotation sequence (from [FRyDOM](http://theory.frydom.org/src/multibody_dynamics/frame.html#diebel))*

In the above rotation sequence, the first rotation of angle, 𝜓, is about the *z-axis*, the second rotation of angle, 𝜃, is about the *y<sup>'</sup><sup>'</sup><sup>'</sup>- axis* with the final rotation of angle, 𝜙, about the *x<sup>'</sup><sup>'</sup>-axis*. This sequence is implemented with the use of rotation matrices. Each rotation in the sequence is expressed as a rotation matrix and the matrix multiplication of the three rotation matrices results in the new orientation, *x<sup>'</sup><sup>'</sup><sup>'</sup>y<sup>'</sup><sup>'</sup><sup>'</sup>z<sup>'</sup><sup>'</sup><sup>'</sup>*.

Consider the following [instance](https://en.wikipedia.org/wiki/Rotation_matrix#General_rotations), 
<p float="center">
  <img title='Rotation matrix' src=images/rotation_matrix.jpg>
</p>

The rotation sequence is ZYX as shown by the matrix multiplication of the matrices *R<sub>z</sub>* with angle 𝛼 (yaw), *R<sub>y</sub>*, with angle 𝛽 (pitch), and *R<sub>x</sub>* with angle 𝛾 (roll). *R* is the product of the rotation matrices. 

The three-angle representation is a consequence of the Euler's rotation theorem which states:
> Any two independent orthonormal coordinate frames can be a sequence of rotations(not more than three) about coordinate axes, where no two successive rotations may be about the same axis.

Representing 3D rotations in this format is natural and intuitive, however, this format runs into the problem of gimbal lock where one degree of freedom is lost because two out of the three coordinate axes are parallely aligned at certain points. Considering a rotation sequence of *ZYX*, rotating the y-axis by +/- 90 degrees (or any multiples of 90) will cause the outer axes of the sequence, z-axis and x-axis, to be aligned and rotation about these axes will have the same effect; eseentially, rotations about these axes are indistinguisable. This problem can be viewed [graphically](https://www.youtube.com/watch?v=mcDHDfK2pXs) and proven [mathematically](https://en.wikipedia.org/wiki/Gimbal_lock) as well. Quaternions are used to avoid this issue of gimbal lock.

#### Quaternions

### MPU6050 Angle Outputs
In the `MPU6050_DMP6.ino` sample file of Jeff Rowberg's library, it can be observed that the MPU6050 sensor is able to output readings in *quaternions*, *euler angles* or *yaw-pitch-roll* angles.

The following lines of code in the main loop confirm these output options (at the time of this project write-up):
```
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Yaw/Pitch/Roll angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif
```

The latest data packet of the MPU6050 module is read from the First-In-First-Out (FIFO) variable, `fifoBuffer` as shown in the first line of the block of code above. The quaternion values are obtained from this variable and the other output methods -- Euler and Yaw-Pitch-Roll angles -- are calculated from the quaternion readings.

The Yaw-Pitch-Roll (YPR) output option is equivalent to implementing a ZYX Cardan rotation sequence, which can be confirmed by the `dmpGetYawPitchRoll()` function definition in the `MPU6050_6Axis_MotionApps20.cpp` file (at the time of this write-up) of the MPU6050 library. However, it was difficult to ascertain the rotataion sequence employed for the Euler angle option.

The YPR output option is best suited for this project as the underlying rotation sequence involves rotation about all three axes. Furthermore, the individual axes can be mapped directly to a respective servo motor: one motor for the z-axis, one for the y-axis and one for the x-axis; this mapping will be elaborated on in a later subsection.

The screenshots below show a sample of the different angle output options, from the MPU6050, observed in the Arduino IDE serial monitor.

<p float="center">
  <img title='Sample output 1' src=images/sample_output_1.png>
  <img title='Sample output 2' src=images/sample_output_2.png>
</p>

### MPU6050 Offsets and Calibration
Sensors are used to measure various physical quantities. As stated earlier, the MPU6050 sensor measures angular velocity and acceleration of an object or system it's mounted on or housed in. Ideally sensors are precise, to produce the same output for the same input, and able to reliably detect minor changes in the measured parameter. However, in the real world sensors aren't perfect. Even sensors produced by the same manufacturer aren't guaranteed to function uniformly; sensor tolerances, made available in datasheets by manufacturers, reflect this potential variation.

Placing an MPU6050 module still and horizontally flat, then reading the raw angular velocity and acceleration values will most likely output nonzero values instead of zeros as one might expect. These variations are called *sensor noise* and can be due to a multitude of different factors. Nonetheless, it's much easier to filter out the noise mathematically rather than correct each of these multitude of factors. 

The observed nonzero values are known as the *sensor offsets*. But before filtering out these offsets, they need to be scaled appropriately to the desired sensor sensitivity. The subsequent subsection details the process of calculating the sensor offsets while accounting for this sensitivity.

#### Calculating MPU6050 offsets

#### Calibration

## Working principle
 
### Sketch Upload

<p float="center">
  <img src=images/sketch_upload.png width="600">
</p>


## Video demonstration

## Observations

## Future work/suggestions
- Smaller voltage regulator for tighter fit
- Use a smaller battery
- connect directly to Vin

## References

- [Additional information on i2cdevlib](https://www.i2cdevlib.com)

- [Get Orientation with Arduino and MPU6050](https://www.teachmemicro.com/orientation-arduino-mpu6050/)

- [Beginner's guide to IMU](https://embeddedinventor.com/what-is-an-imu-sensor-a-complete-guide-for-beginners/)

- [Offset calculations](https://forum.arduino.cc/t/calculating-offsets-for-mpu6050-gyro/514661/2)

- https://randomnerdtutorials.com/arduino-mpu-6050-accelerometer-gyroscope/

- [MPU6050 Accelerometer and Gyroscope Module](https://components101.com/sensors/mpu6050-module)

- [3D Rotations](https://www.euclideanspace.com/maths/geometry/rotations/index.htm)

- [Calibrating sensors](https://learn.adafruit.com/calibrating-sensors?view=all)