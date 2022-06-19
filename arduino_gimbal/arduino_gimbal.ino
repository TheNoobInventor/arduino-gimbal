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
volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

#define INTERRUPT_PIN 2  
MPU6050 accelgyro;      // Instantiate an MPU6050 object

// Servo definitions
Servo servoYaw;
Servo servoPitch;
Servo servoRoll;

void setup() {
  
  // Initialize Wire library and join i2c bus
  Wire.begin();
  Wire.setClock(400000); 
 
  // Initialize serial communication
  Serial.begin(38400);

  // Initialize device
  Serial.println(F("Initializing I2C devices..."));
  accelgyro.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // Verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(accelgyro.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // Load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = accelgyro.dmpInitialize();

  // Set device offsets obtained from mpu6050_offsets.ino sketch
  accelgyro.setXGyroOffset(-203);
  accelgyro.setYGyroOffset(-95); 
  accelgyro.setZGyroOffset(-48); 
  accelgyro.setZAccelOffset(1933); 
  
  // Make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    
    // Calibrate and fine tune MPU6050 with the offsets set above
    accelgyro.CalibrateAccel(6);
    accelgyro.CalibrateGyro(6);
    accelgyro.PrintActiveOffsets();
    
    // Turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    accelgyro.setDMPEnabled(true);

    // Enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = accelgyro.getIntStatus();

    // Set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // Get expected DMP packet size for later comparison
    packetSize = accelgyro.dmpGetFIFOPacketSize();
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

  // If programming failed, don't try to do anything
  if (!dmpReady) return;
  
  // Read a packet from FIFO
  if (accelgyro.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    
    // Display Euler angles in degrees
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Convert Yaw, Pitch and Roll values from Radians to degrees
    ypr[0] = ypr[0] * 180 / M_PI;
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;

    /* Map the MPU6050 movement to the angular movements of the servo motors. This mapping has to be done for the servos
       to move in the opposite direction of the MPU6050 accelerometer motion, for each respective axis, to attempt
       stabilizing the gimbal platform. 
    */
    int yawValue = map(ypr[0], -90, 90, 0, 180);            
    int pitchValue = map(ypr[1], -90, 90, 180, 0);            
    int rollValue = map(ypr[2], -90, 90, 180, 0);

    // Control servos according to the MPU6050 orientation
    servoYaw.write(yawValue);
    servoPitch.write(pitchValue);
    servoRoll.write(rollValue);
  }
}
