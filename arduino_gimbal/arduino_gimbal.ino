/* 
 * A 3D printed DIY gimbal - fitted with 3 FS90 Micro servo motors - is controlled using an MPU6050 
 * inertial measurement unit (IMU) sensor in order to keep 3D printed platform stable during rotation in 
 * any axis. 
 * 
 * All of these components are connected to an Arduino Nano and powered via a 9V battery - stepped down 
 * to 5V using a buck converter.
 * 
 */

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Servo.h>

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Interrupt detection routine
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

#define INTERRUPT_PIN 2  
MPU6050 mpu;

// Servo definitions
Servo servoYaw;
Servo servoPitch;
Servo servoRoll;

void setup() {
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
   
    // initialize serial communication
    Serial.begin(38400);//

    // Initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Offsets obtained from calibration file (mpu6050_offsets.ino)
    mpu.setXGyroOffset(-209);
    mpu.setYGyroOffset(-100); 
    mpu.setZGyroOffset(-50); 
    mpu.setZAccelOffset(-2153); 
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // Attach servos to digital pins
    servoYaw.attach(5);
    servoPitch.attach(10);
    servoRoll.attach(4);
}

void loop() {

    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Convert Yaw, Pitch and Roll values from Radians to degrees
        ypr[0] = ypr[0] * 180 / M_PI;
        ypr[1] = ypr[1] * 180 / M_PI;
        ypr[2] = ypr[2] * 180 / M_PI;

        // Summarize and add to docs:
        // When we move the mpu6050 accelerometer sensor then the data related to its movement is mapped 
        // with the angle of movement of servo. Hence, the servo motor will move with respect to the degree
        // of movement of the accelerometer.
        
//          Now, to stabilize our platform we have to move the servo opposite to the direction of the accelerometer 
//          sensor motion. That means when we try to move our platform in one direction, the servo shaft will move 
//          in the other direction. In this way, the camera or our platform stays in its original position irrespective 
//          of the direction the accelerometer sensor moves

        // Reverse map value of servo if the servo moves
        // in the same direction of motion
        
        int yawValue = map(ypr[0], -90, 90, 0, 180);            
        int pitchValue = map(ypr[1], -90, 90, 180, 0);            
        int rollValue = map(ypr[2], -90, 90, 180, 0);

        // Control servos according to the MPU6050 orientation
        servoYaw.write(yawValue);
        servoPitch.write(pitchValue);
        servoRoll.write(rollValue);
    }
}
