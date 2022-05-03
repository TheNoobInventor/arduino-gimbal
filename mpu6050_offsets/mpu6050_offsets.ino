/*
 * Adapted from MPU6050_raw example file to generate offsets required to calibrate the MPU6050 for gimbal use.
 */

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;

int16_t rawValues[6]; // for what axes? explain succintly {ax, ay, az, gx, gy, gz};
int16_t totalOffset[4];
int16_t averageOffset[4];
int16_t calculatedOffset[4] = {0, 0, 0, 0};
int16_t counter = 1;
bool isOffsetCalculated = false; //

void setup() {
    //
    Wire.begin();

    // Initialize serial communication
    Serial.begin(38400);

    // Initialize device
    accelgyro.initialize();

    // Verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // Set the offset values of ZAccel, XGyro, YGyro and ZGyro to the initial values of calculatedOffset: {0, 0, 0, 0}     
    setOffsetValues();
    
}

void loop() {
  
    // Read raw accel/gyro measurements from device
    accelgyro.getMotion6(&rawValues[0], &rawValues[1], &rawValues[2], &rawValues[3], &rawValues[4], &rawValues[5]);

    //
    if(isOffsetCalculated == false){
      calculateOffsets();
    }
     counter ++;
}

/*
 * 
 */
void setOffsetValues(){

    //
    accelgyro.setZAccelOffset(calculatedOffset[0]);
    accelgyro.setXGyroOffset(calculatedOffset[1]);
    accelgyro.setYGyroOffset(calculatedOffset[2]); 
    accelgyro.setZGyroOffset(calculatedOffset[3]); 

    if(isOffsetCalculated == true){
      Serial.print(calculatedOffset[0]); Serial.print("\t");
      Serial.print(calculatedOffset[1]); Serial.print("\t");
      Serial.print(calculatedOffset[2]); Serial.print("\t");
      Serial.print(calculatedOffset[3]); Serial.print("\t"); 
    }

}

/* Calculate sensor offsets. 
 *  
 *  The offsets are calculated using the average value for each parameter over 30 readings. 
 */
void calculateOffsets(){
  
    if(counter <= 30){
       for(int i = 0, j = 2; i<4, j<6; i++, j++){
          totalOffset[i] = totalOffset[i] + rawValues[j];
          averageOffset[i] = totalOffset[i] / counter;
          
        if(counter == 30){
 
        /*
         * Set offset values of gyro and accel
         * 
         * Explain how offsets are calculated
         */
          calculatedOffset[0] = (averageOffset[0] - 16384)/ 8;
          calculatedOffset[1] = -averageOffset[1] / 4;
          calculatedOffset[2] = -averageOffset[2] / 4;
          calculatedOffset[3] = -averageOffset[3] / 4;
  
          isOffsetCalculated = true;
          setOffsetValues();
        }
      } 
    }
}

// TO-DO: Calculate offsets of x and y accelerations for a project that requires those parameters.
