/*
  BMI270_AUX_BMM150 - Simple Accelerometer

  This example reads the acceleration values from the BMI270
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

History:
 Riccardo Rizzo - created 10 Jul 2019
 Yahya Tawil - ported from Arduino BMI270 library to BMI270_AUX_BMM150  13 May 2023 

  This example code is in the public domain.
*/

#include "BMI270_AUX_BMM150.h"

BMI2_BMM1_Class imu(Wire);

uint32_t sample_rate = 0;
uint8_t new_mag_data = 0;
uint8_t new_accel_data = 0;
uint8_t new_gyro_data = 0;
uint32_t dt = 0;
elapsedMillis em;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

/******************************************************************************/
/*! @name       Accelerometer Macro Definitions               */
/******************************************************************************/
/*! @name Accelerometer Bandwidth parameters */
/*
#define BMI2_ACC_OSR4_AVG1                        UINT8_C(0x00)
#define BMI2_ACC_OSR2_AVG2                        UINT8_C(0x01)
#define BMI2_ACC_NORMAL_AVG4                      UINT8_C(0x02)
#define BMI2_ACC_CIC_AVG8                         UINT8_C(0x03)
#define BMI2_ACC_RES_AVG16                        UINT8_C(0x04)
#define BMI2_ACC_RES_AVG32                        UINT8_C(0x05)
#define BMI2_ACC_RES_AVG64                        UINT8_C(0x06)
#define BMI2_ACC_RES_AVG128                       UINT8_C(0x07)
*/
/*! @name Accelerometer Output Data Rate */
/*
#define BMI2_ACC_ODR_0_78HZ                       UINT8_C(0x01)
#define BMI2_ACC_ODR_1_56HZ                       UINT8_C(0x02)
#define BMI2_ACC_ODR_3_12HZ                       UINT8_C(0x03)
#define BMI2_ACC_ODR_6_25HZ                       UINT8_C(0x04)
#define BMI2_ACC_ODR_12_5HZ                       UINT8_C(0x05)
#define BMI2_ACC_ODR_25HZ                         UINT8_C(0x06)
#define BMI2_ACC_ODR_50HZ                         UINT8_C(0x07)
#define BMI2_ACC_ODR_100HZ                        UINT8_C(0x08)
#define BMI2_ACC_ODR_200HZ                        UINT8_C(0x09)
#define BMI2_ACC_ODR_400HZ                        UINT8_C(0x0A)
#define BMI2_ACC_ODR_800HZ                        UINT8_C(0x0B)
#define BMI2_ACC_ODR_1600HZ                       UINT8_C(0x0C)
*/
/*! @name Accelerometer G Range 
#define BMI2_ACC_RANGE_2G                         UINT8_C(0x00)
#define BMI2_ACC_RANGE_4G                         UINT8_C(0x01)
#define BMI2_ACC_RANGE_8G                         UINT8_C(0x02)
#define BMI2_ACC_RANGE_16G                        UINT8_C(0x03)
*/
  imu.setAcellConfig(BMI2_ACC_ODR_100HZ, BMI2_ACC_RANGE_16G, BMI2_ACC_NORMAL_AVG4);

/*! @name Gyroscope Bandwidth parameters
#define BMI2_GYR_OSR4_MODE                        UINT8_C(0x00)
#define BMI2_GYR_OSR2_MODE                        UINT8_C(0x01)
#define BMI2_GYR_NORMAL_MODE                      UINT8_C(0x02)
#define BMI2_GYR_CIC_MODE                         UINT8_C(0x03)
*/
/*! @name Gyroscope Output Data Rate
#define BMI2_GYR_ODR_25HZ                         UINT8_C(0x06)
#define BMI2_GYR_ODR_50HZ                         UINT8_C(0x07)
#define BMI2_GYR_ODR_100HZ                        UINT8_C(0x08)
#define BMI2_GYR_ODR_200HZ                        UINT8_C(0x09)
#define BMI2_GYR_ODR_400HZ                        UINT8_C(0x0A)
#define BMI2_GYR_ODR_800HZ                        UINT8_C(0x0B)
#define BMI2_GYR_ODR_1600HZ                       UINT8_C(0x0C)
#define BMI2_GYR_ODR_3200HZ                       UINT8_C(0x0D)
*/
/*! @name Gyroscope Angular Rate Measurement Range
#define BMI2_GYR_RANGE_2000                       UINT8_C(0x00)
#define BMI2_GYR_RANGE_1000                       UINT8_C(0x01)
#define BMI2_GYR_RANGE_500                        UINT8_C(0x02)
#define BMI2_GYR_RANGE_250                        UINT8_C(0x03)
#define BMI2_GYR_RANGE_125                        UINT8_C(0x04)
*/
  imu.setGyroConfig(BMI2_GYR_ODR_100HZ, BMI2_GYR_RANGE_2000, BMI2_GYR_NORMAL_MODE);

/*! @name PRESET MODE DEFINITIONS 
#define BMM150_PRESETMODE_LOWPOWER                UINT8_C(0x01)
#define BMM150_PRESETMODE_REGULAR                 UINT8_C(0x02)
#define BMM150_PRESETMODE_HIGHACCURACY            UINT8_C(0x03)
#define BMM150_PRESETMODE_ENHANCED                UINT8_C(0x04)
 */
 imu.setMagConfig(BMM150_POWERMODE_NORMAL, BMM150_PRESETMODE_HIGHACCURACY);
  if (!imu.begin()) {
    Serial.println("Failed to initialize imu!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(imu.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.print("Gyroscope sample rate = ");
  Serial.print(imu.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.print("Magnetic field sample rate = ");
  Serial.print(imu.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's\tGyroscope in degrees/second\tMagnetic Field in uT");
  Serial.println("X\tY\tZ\tX\tY\tZ\tX\tY\tZ");

  sample_rate = 1000000.0f/imu.accelerationSampleRate();

  Serial.println(sample_rate,3);
  
}

void loop() {
  float values[9];

  readimu(values);

  if(new_accel_data && new_gyro_data) {
    dt = em;
    Serial.printf("%d, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", dt, values[0], values[1], 
                values[2], values[3], values[4], values[5], values[6], values[7], values[8]);
    em = 0;
  }
}

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
void readimu(float *values) {
  new_mag_data = 0;
  new_accel_data = 0;
  new_gyro_data = 0;

  if(imu.accelerationAvailable()) {
    imu.readAcceleration(ax, ay, az);
    new_accel_data = 1;
  }
    values[0] = ax;
    values[1] = -ay;
    values[2] = -az;

  if(imu.gyroscopeAvailable()) {
    imu.readGyroscope(gx, gy, gz);
    new_gyro_data = 1;
  }
    values[3] = gx;
    values[4] = -gy;
    values[5] = -gz;

  if (imu.magneticFieldAvailable()) {
    imu.readMagneticField(mx, my, mz);
    new_mag_data = 1;
  }
    values[6] = my;
    values[7] = -mx;
    values[8] = mz;
}
