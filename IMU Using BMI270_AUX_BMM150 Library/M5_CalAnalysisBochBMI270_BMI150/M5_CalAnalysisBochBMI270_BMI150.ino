/*

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//////////////////////////////////////////////////////////////////////////////////////
Calibration Instructions and Notes.

Gyro bias calibration is typically accomplished each time the IMU initializes (powers on).
Accelerometer and magnetometer bias and scale factor calibration is typically accomplished 
prior to IMU operations, and the values are stored in EEPROM. 

This program performs calibration for gyro bias, accelerometer bias and scale factor, and
magnetometer bias and scale factor. The gyro bias calibration is really just provided so
that user can verify it will perform as expected upon IMU startup. The accel and mag
calibrations are written to EEPROM where they can be retrieved upon IMU startup.
////////////////////////////////////////////////////////////////////////////////////
*/
#include <M5Unified.h>
#include <elapsedMillis.h>
#include "BMI270_AUX_BMM150.h"
BMI2_BMM1_Class IMU(Wire1);

uint32_t sample_rate = 0;
uint8_t new_mag_data = 0;
uint8_t new_accel_data = 0;
uint8_t new_gyro_data = 0;
uint32_t dt = 0;
elapsedMillis em;

// IMU Declares
#define IMU_BUS       Wire  //Wire // SPI
#define IMU_SPD       400000  //1000000 // 0==NULL or other


// EEPROM buffer and variable to save accel and mag bias and scale factors
float value;  // temp variable for loading into EEPROM buffer
char rx_byte = 0;
float axb, axs=1.0f, ayb, ays=1.0f, azb, azs=1.0f;
float hxb, hxs=1.0f, hyb, hys=1.0f, hzb, hzs=1.0f;
float gxb, gyb, gzb;
bool serialPrintFlag = false;
const float G = -9.80665;
#define deg2rad PI/180

// Set the values below for the circular buffer size (bufSize) and
// the number of loops to execute for passing data to the circular buffer.
const int bufSize = 2048; //64; // Must be power of 2
int numIter = 2070; //70; // Must be greater than bufSize

// Magnetic Field Strength in NED frame for Whitestone, NY
//   Reference: ngdc.noaa.gov
const float hNorth = 20.4631;
const float hEast = -4.5981;
const float hDown = 46.5087 ;

#define G 9.80665f

void setup() {
  // initialize serial to display instructions
  Serial.begin(115200);
  while(!Serial && millis() < 5000) {}
  // start communication with IMU 
  Serial.println(" ");

   auto cfg = M5.config();
   M5.begin(cfg);

  IMU.setAcellConfig(BMI2_ACC_ODR_100HZ, BMI2_ACC_RANGE_16G, BMI2_ACC_NORMAL_AVG4);
  IMU.setGyroConfig(BMI2_GYR_ODR_200HZ, BMI2_GYR_RANGE_2000, BMI2_GYR_NORMAL_MODE);
  IMU.setMagConfig(BMM150_POWERMODE_NORMAL, BMM150_PRESETMODE_REGULAR);

  Serial.println("Beginning IMU Initialization...");

  if (!IMU.begin()) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    while(1) {}
  }
  Serial.println("IMU Initialization Complete");
  Serial.println(" ");


  // Read and print current calibration values
  printIMUBiases();
  
  // tell user the command options
  Serial.println("Enter one of the following commands using the serial terminal: ");
  Serial.println("  Enter 'a' to preform MPU library accel calibrations");
  Serial.println("  Enter 'm' to perform MPU library mag calibrations");
  Serial.println("  Enter 'g' to perform MPU library gyro calibrations");
  Serial.println("  Enter 's' to perform static IMU bias calibrations");
  Serial.println("  Enter 'd' to display all calibration values");
  Serial.println("  Enter 'z' to reset all calibration values to zero");
  Serial.println("  Enter 'p' to print corrected IMU readings to serial");
  Serial.println("  Enter 'r' to calculate IMU sensor noise sigmas");
  Serial.println(" ");
} // end setup loop

void loop() {
  
  // wait for user command
  if (Serial.available()>0) {
    rx_byte = Serial.read();

    // read user command and execute proper routine
    if (rx_byte == 'a'){
      accelerometerCal();
    }
    else if (rx_byte == 'm'){
      magnetometerCal();
    }
    else if (rx_byte == 'g'){
      gyroCal();
    }
    else if (rx_byte == 'd'){
      Serial.println("Printing IMU:"); printIMUBiases();
    }
    else if (rx_byte == 's'){
      staticCal();
    }
    else if (rx_byte == 'p'){
      serialPrintFlag = !serialPrintFlag;
    }
    else if (rx_byte == 'r'){
      noiseLevelsIMU(); // calculate noise sigma for all IMU sensors
    }
  }

  if (serialPrintFlag){
    serialPrint();  
    delay(200); // Update rate for streaming to serial
  }
} // End main void loop


void readIMU(float *values) {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

  new_mag_data = 0;
  new_accel_data = 0;
  new_gyro_data = 0;

  if(IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    new_accel_data = 1;
  }
    values[0] = -ay;
    values[1] = -ax;
    values[2] = -az;

  if(IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    new_gyro_data = 1;
  }
    values[3] = -gy ;
    values[4] = -gx ;
    values[5] = -gz ;

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
    new_mag_data = 1;
  }
  values[6] = mx;
  values[7] = my;
  values[8] = mz;

}

