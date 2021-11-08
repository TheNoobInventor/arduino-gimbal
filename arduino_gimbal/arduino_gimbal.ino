/* 
 * Arduino Nano + 3 servo90 (confirm this) motors + MPU6050
 * ...
 * 
 * 
 */

// Include libraries

#include <Servo.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// MPU control and status variables
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion variables
Quaternion q;          // [w, x, y, z]         quaternion container
VectorFloat gravity;   // [x, y, z]            gravity vector
float ypr[3];          // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/*
 * 
 */
 
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

#define INTERRUPT_PIN 13

//
MPU6050 mpu;

//
Servo servo_yaw;
Servo servo_pitch;
Servo servo_roll;


//===========================================================
// Setup
//===========================================================
void setup() {
  //
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // Initialize serial for debugging purposes
  Serial.begin(115200); 
  while (!Serial);

  mpu.initialize(); // Initialize device
  pinMode(INTERRUPT_PIN, INPUT); //
  devStatus = mpu.dmpInitialize(); //

  // Provide your own offset values for stable output; values obtained iteratively.
  mpu.setXGyroOffset(87); // still work on this
  mpu.setYGyroOffset(20); // okay for now
  mpu.setZGyroOffset(127); // okay for now
  mpu.setZAccelOffset(1450); // tweak later

//  mpu.setXGyroOffset(17);
//  mpu.setYGyroOffset(-69);
//  mpu.setZGyroOffset(80);
//  mpu.setZAccelOffset(1450);

  // for checking dmp device status?
  // make sure it worked (returns 0 if so)
  
  if (devStatus == 0) {
    Serial.println(F("Enabling DMP ..."));
//    /Serial.println("Enabling DMP without F ...");
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    //
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  //
  servo_yaw.attach(5); // digital pin 5
  
  servo_pitch.attach(4);
  servo_roll.attach(3);

//  confirm that servos are attached appropriately
}

//==============================================================
// Main loop
//=============================================================
void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    //
    readMPU6050();

    /*
     * check rf_receiver sketch for some reference
     * 
     */
    
    // Have to tune servo directions to your needs !!!!
    int yawValue = map(ypr[0], -90, 90, 0, 180);
    int pitchValue = map(ypr[1], -90, 90, 180, 0);
    int rollValue = map(ypr[2], -90, 90, 0, 180);

    // Understand and confirm these
    servo_yaw.write(min(180, max(0, yawValue)));
    servo_pitch.write(min(180, max(0, pitchValue)));
    servo_roll.write(min(180, max(0, rollValue)));
//
//    Serial.print("yaw : ");
//    Serial.print(yawValue);
//    Serial.print(", pitch : ");
//    Serial.print(pitchValue);
//    Serial.print(", roll : ");
//    Serial.println(rollValue);

}


// Understand this function below and comment accordingly
// make parts my own too

//float correct; // why float?
float correct[3];// mine
int i = 0;

//void readMPU6050() {

  // wait for MPU interrupt or extra packet(s) available
  
//  while (!mpuInterrupt && fifoCount < packetSize) {
//    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop 
//      fifoCount = mpu.getFIFOCount();
//      Serial.print("in readMPU6050 function");
//    }
//  }

// check commented section above

void readMPU6050() {
    if (mpuInterrupt && fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
      Serial.print("in readMPU6050 function");
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
      // reset so that we can continue cleanly
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));

    // otherwise check for DMP data ready interrupt (this should happen frequently)ad
  } 
  else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display euler angles in degrees (is this correct?)
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    ypr[0] = ypr[0] * 180 / M_PI;
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;

    if (i <= 300) {
      i++;
      //correct = ypr[0];
      // mine
      correct[0] = ypr[0];
      correct[1] = ypr[1];
      correct[2] = ypr[2];
      readMPU6050();
      return;
    }
      
    ypr[0] = ypr[0] - correct[0];
    ypr[1] = ypr[1] - correct[1];
    ypr[2] = ypr[2] - correct[2];
  }  

  
//  why not ypr[1] and ypr[2]?/

// roll keeps drifting

// wildly spinning on yaw when i move gimbal too far in one direction

}


/*
 * 
 * /** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 *
 * mpu.initialize()
 * 
 * 
 * mpu.dmpInitialize()
 * mpu.setXGyroOffset(17)
 * mpu.setDMPEnabled(true)
 * attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING)
 * 
 * /** Get full set of interrupt status bits.
 * These bits clear to 0 after the register has been read. Very useful
 * for getting multiple INT statuses, since each single bit read clears
 * all of them because it has to read the whole byte.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 * 
 * mpu.getIntStatus()
 * 
 *
 * 
 * The DMP, or Digital Motion Processor, is an internal processing unit contained within the MPU-6050 and its successors 
 * (MPU-6150, MPU-9150, possibly more in the future). The processor has been described by InvenSense employees as a sort of limited CPU, 
 * or alternatively as an enhanced ALU (arithmetic logic unit), which is built with an instruction set designed for very specific 3D math 
 * operations necessary for orientation calculation. There is currently no known resource for understanding what this instruction set is or how it works.
 * 
 * In order to properly use the DMP instead of just the raw sensor output, the following basic operations must be done after the device is powered on and 
 * sleep mode is disabled:
    Load a block of DMP binary code into volatile MPU memory banks (takes ~1 second)
    Apply a set of DMP configuration settings to the appropriate MPU memory bank locations
    Configure desired rate/sensitivity/interrupt settings in MPU registers
    Enable DMP
    Read FIFO on DMP_INT detection
This is just a basic overview of the steps required, and does not describe every individual bit of configuration necessary
 * 
 * The DMP is capable of generating 6-axis orientation data in the form of a quaternion, which is a special vector/rotation matrix in the form of 
 * [w x y z], where each component is a value between -1 and +1. Using the DMP binary code block from InvenSense' published MotionApps 2.0 platform, 
 * the quaternion is scaled to a 16-bit or integer representation, and more specifically into a signed 15-bit integer between -16384 and +16383 for 
 * each of the four elements. The DMP uses the MPU's internal FIFO to output this data, and can send raw accelerometer and gyroscope values at the same 
 * time as well. The easiest way to achieve 6-axis DMP output without using the full MotionApps codebase from InvenSense is with the MPU6050_DMP6.info 
 * example sketch for the Arduino.
 * 
 */
