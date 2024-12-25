/*
Madgwick Fusion Library:
	The MIT License (MIT)
	Copyright (c) 2021 x-io Technologies

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <M5Unified.h>
//#include "elapsedMillis.h"
#include "Arduino.h"


#include "Fusion.h"
#include <stdbool.h>
#include <stdio.h>

#include <string>

#include "constants.h" 

void gyroCalibration();

void telemetryPortOut();
void print_float_array(float *arr, int size);

float acc_off[] = {0.090992, 0.090992, 0.122667};
float acc_scale[]  = {1.004763, 0.997774, 0.994054};

uint8_t new_mag_data = 0;
uint8_t new_accel_data = 0;
uint8_t new_gyro_data = 0;

FusionOffset offset;
FusionAhrs ahrs;
FusionQuaternion q;
FusionEuler euler;
FusionAhrsFlags flags;

// Define calibration (replace with actual calibration data if available)
  const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
  FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};

  const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  const FusionVector accelerometerSensitivity = {1.001222, 1.003922, 0.997322};
  const FusionVector accelerometerOffset = {0.046889, 0.046889, 0.040066};

  const FusionMatrix softIronMatrix = {0.9603,0.0076,0.0104,0.0076,0.9566,-0.0132,0.0104,-0.0132,1.0890};
  const FusionVector hardIronOffset = {12.416,19.487,-8.849};

//new data available
volatile int newIMUData;
uint32_t lastUpdate, now1;


void setup() {
   auto cfg = M5.config();
   M5.begin(cfg);
   M5.Imu.begin();

  /* Serial to display data */
  while(!Serial && millis() < 5000) {}
  Serial.begin(115200);
  

  Serial.println("Beginning IMU Initialization...");

  if(!M5.Imu.isEnabled()){
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    while(1) {}
  }
  Serial.println("IMU Initialization Complete");
  Serial.println(" ");

  Serial.println("IMU Connected!");
  delay(10);

  SAMPLE_RATE = 0.012f;  //gyro sample rate

  FusionOffsetInitialise(&offset, SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);
    
  // Set AHRS algorithm settings
  const FusionAhrsSettings settings = {
          .convention = FusionConventionNed,
          .gain = 1.5f, //1.5f,
          .gyroscopeRange = 2000.0f, // replace this with actual gyroscope range in degrees/s
          .accelerationRejection = 10.0f,
          .magneticRejection = 10.0f, //0.0f,
          .recoveryTriggerPeriod  = (uint8_t)(5.0f / SAMPLE_RATE ), // 5 seconds
  };

  FusionAhrsSetSettings(&ahrs, &settings);

  gyroCalibration();

  //Serial.println("Ready for commands....");
  telem_values_on = 0;
  raw_values_on = 0;
  x_values_on = 1;
  fusion_on = 1;
}

void loop() {
/*    if ( Serial.available() ) {
      char rr;
      rr = Serial.read();
      switch (rr) {
        case 'g':
          rr = '0';
          gyroCalibration();
          break;
        case 'r':
        {
          rr ='0';
          if(raw_values_on == 1) {
            raw_values_on = 0;
            Serial.println("RAW DATA Off!");
          } else if(raw_values_on == 0) {
            Serial.println("RAW DATA ON!");
            raw_values_on = 1;
          }
        }
          break;
        case 's':
        {
          rr ='0';
          if(serial_values_on == 1) {
            serial_values_on = 0;
            Serial.println("Serial Data Off!");
          } else if(serial_values_on == 0) {
            Serial.println("Serial Data ON!");
            serial_values_on = 1;
          }
        }
          break;
        case 'x':
        {
          rr ='0';
          if(x_values_on == 1) {
            x_values_on = 0;
            Serial.println("Ximu DATA OFF!");
          } else if(serial_values_on == 0) {
            Serial.println("Ximu DATA ON!");
            x_values_on = 1;
          }
        }
            break;
        case 't':
        {
          rr ='0';
          if(telem_values_on == 1) {
            telem_values_on = 0;
            Serial.println("Telem Off!");
          } else if(telem_values_on == 0) {
            Serial.println("Telem ON!");
            telem_values_on = 1;
          }
        }
            break;
        case 'f':
        {
          rr = '0';
          if(fusion_on == 1) {
            Serial.println("Fusion Off!");
            fusion_on = 0;
          } else if(fusion_on == 0) {
            Serial.println("Fusion ON!");
            fusion_on = 1;
          }
        }
          break;
        case '\r':
        case '\n':
        case 'h': menu(); break;
      }
      while (Serial.read() != -1) ; // remove rest of characters.
    } 
*/
  if(fusion_on == 1) {
    getFusion();
  }
}


void getCalIMU() {
  // read the sensor
   float ax, ay, az;
   float gx, gy, gz;
   float mx, my, mz;

   M5.Imu.getAccel(&ax, &ay, &az);
   M5.Imu.getGyro(&gx, &gy, &gz);
   M5.Imu.getMag(&mx, &my, &mz);
   
   // Match axis orientation from example
   val[0] = -ay;
   val[1] = -ax;
   val[2] = -az;
   
   val[3] = -gy ;
   val[4] = -gx ;
   val[5] = -gz ;
   
   val[6] = mx / 10.0f;
   val[7] = my / 10.0f;
   val[8] = mz / 10.0f;
   
   new_mag_data = true;

  if(new_mag_data) {
    now1= micros(); // This is when the data reported READY
    //Serial.printf(" %f, %f, %f, %f, %f, %f, %f, %f, %f\n", val[0], val[1], 
    //            val[2], val[3], val[4], val[5], val[6], val[7], val[8]);
  }

}


void getFusion() {
  getCalIMU();

  if(new_mag_data) {
    dt = ((now1 - lastUpdate) * 0.000001);
    lastUpdate = now1;

    //Lets normalize magnetometer data
    //float mag_norm = sqrt(val[6]*val[6] + val[7]*val[7] + val[8]*val[8]);

    // Acquire latest sensor data
    FusionVector gyroscope = { val[3], val[4], val[5] }; // replace this with actual gyroscope data in degrees/s
    FusionVector accelerometer = { val[0], val[1], val[2] }; // replace this with actual accelerometer data in g
    FusionVector magnetometer = {  val[6], val[7], val[8] }; // replace this with actual magnetometer data in arbitrary units

    // Apply calibration
    //gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
    magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

    // Update gyroscope offset correction algorithm
    //gyroscope = FusionOffsetUpdate(&offset, gyroscope);

    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, dt);
    //FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dt);

    flags = FusionAhrsGetFlags(&ahrs);
    euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    q = FusionAhrsGetQuaternion(&ahrs);
    fHeading = FusionCompassCalculateHeading(FusionConventionNed, accelerometer, magnetometer);


    //if(dump > 100) {
      if(raw_values_on == 1) {
        Serial.println(dt, 8);
        print_float_array(val, 9);
        Serial.println();
      }

      if(serial_values_on == 1) {
        Serial.printf(" %0.1f, %0.1f, %0.1f, %0.1f\n", fHeading, euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
      }
  
      if(x_values_on == 1) {
        timestamp = micros();
        char accelgyroBuffer[100];
        sprintf(accelgyroBuffer, "%c,%llu,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f", 'I', timestamp, val[3], val[4], val[5], val[0], val[1], val[2]);
        Serial.printf ("%s\n",accelgyroBuffer);
        sprintf(accelgyroBuffer, "%c,%llu,%0.6f,%0.6f,%0.6f", 'M', timestamp, magnetometer.axis.x, magnetometer.axis.y, magnetometer.axis.z);
        Serial.printf ("%s\n",accelgyroBuffer);
        sprintf(accelgyroBuffer, "%c,%llu,%0.6f,%0.6f,%0.6f,%0.6f", 'Q', timestamp, q.element.w, q.element.x, q.element.y, q.element.z);
        Serial.printf ("%s\n",accelgyroBuffer);
      }

      if(telem_values_on == 1) telemetryPortOut();
      //dump = 0;

      delay(10);
  }
}

void gyroCalibration() {
  int numSamples = 500;
  float gx, gy, gz;
  gyrox_off = 0;
  gyroy_off = 0;
  gyroz_off = 0;
  //imu.DisableDrdyInt();
  uint16_t icount = 0;
  while(icount < numSamples){
   M5.Imu.getGyro(&gx, &gy, &gz);
      icount += 1;
      gyrox_off += gx;
      gyroy_off += gy;
      gyroz_off += gz;
  }
  gyrox_off = gyrox_off / numSamples;
  gyroy_off = gyroy_off / numSamples;
  gyroz_off = gyroz_off / numSamples;
  Serial.printf("%f, %f, %f\n", gyrox_off, gyroy_off,gyroz_off );
  gyroscopeSensitivity = {1.0, 1.0, 1.0};
  gyroscopeOffset = {gyrox_off, gyrox_off, gyrox_off};
}
/*
void menu()
{
  Serial.println();
  Serial.println("Menu Options:");
  Serial.println("========================================");
  Serial.println("\tx - x-IMU3 GUI Output");
  Serial.println("\tt - Telemetry Viewer Output");
  Serial.println("\ts - Serial Print Output (Euler Angles)");
  Serial.println("\tf - Fusion On");
  Serial.println("\tr - Print Values");
  Serial.println("========================================");
  Serial.println("\tg - Zero Gyroscope");

  Serial.println("========================================");
  Serial.println("\th - Menu");
  Serial.println("========================================");
  Serial.println();
}
*/