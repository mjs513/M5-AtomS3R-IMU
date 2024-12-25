#include <M5Unified.h>
#include "Arduino.h"
#include "elapsedMillis.h"
#include "BMI270_AUX_BMM150.h"

uint32_t time_old = 0;

#include "crc16.h"

uint32_t sample_rate = 0;
uint8_t new_mag_data = 0;
uint8_t new_accel_data = 0;
uint8_t new_gyro_data = 0;
float deg2rad = PI / 180.0f;

const int ledPin = LED_BUILTIN;
int ledState = LOW;
int ledFastblinks = 0;
elapsedMillis ledMillis = 0;
int loopcount = 0;
void receiveCalibration();

void setup() {
   auto cfg = M5.config();
   M5.begin(cfg);

  Serial.begin(115200);
  while (!Serial) ; // wait for serial port open
  delay(800);

  if(!M5.Imu.isEnabled()) {
    Serial.println("Failed to initialize IMU");
    while(1);
  }
  Serial.println("IMU Initialization Complete");
  Serial.println(" ");

  Serial.println("IMU Connected!");

  //pinMode(ledPin, OUTPUT);
}


float accel_zerog[3], gyro_zerorate[3], mag_hardiron[3], mag_softiron[9];
float magfield, mag_field;

void loop() {
  float raw_values[9];

  // get and print uncalibrated data
  getCalIMU(raw_values);
  
    if(new_mag_data){
      //imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
      Serial.print("Raw:");
      Serial.print(int(raw_values[0] * 8192));
      Serial.print(',');
      Serial.print(int(raw_values[1] * 8192));
      Serial.print(',');
      Serial.print(int(raw_values[2] * 8192));
      Serial.print(',');
      Serial.print(int (raw_values[3] * 16 ));
      Serial.print(',');
      Serial.print(int (raw_values[4] * 16 ));
      Serial.print(',');
      Serial.print(int (raw_values[5] * 16 ));
      Serial.print(',');
      Serial.print(int (raw_values[6]));  //already in uT
      Serial.print(',');
      Serial.print(int (raw_values[7]));
      Serial.print(',');
      Serial.print(int (raw_values[8]));
      Serial.println("");
      loopcount = loopcount + 1;
    }

  // check for incoming calibration
  receiveCalibration();

  // occasionally print calibration
  if (loopcount == 50 || loopcount > 100) {
    Serial.print("Cal1:");
    for (int i=0; i<3; i++) {
      Serial.print(accel_zerog[i], 3); 
      Serial.print(",");
    }
    for (int i=0; i<3; i++) {
      Serial.print(gyro_zerorate[i], 3);
      Serial.print(",");
    }  
    for (int i=0; i<3; i++) {
      Serial.print(mag_hardiron[i], 3); 
      Serial.print(",");
    }  
    Serial.println(mag_field, 3);
    loopcount++;
  }
  if (loopcount >= 100) {
    Serial.print("Cal2:");
    for (int i=0; i<9; i++) {
      Serial.print(mag_softiron[i], 4); 
      if (i < 8) Serial.print(',');
    }
    Serial.println();
    loopcount = 0;
  }

  //blink LED, slow normally, fast when calibration written
  if (ledMillis >= 1000) {
    if (ledFastblinks > 0) {
      ledFastblinks = ledFastblinks - 1;
      ledMillis -= 125;
    } else {
      ledMillis -= 1000;
    }
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(ledPin, ledState);
  }
 delay(10);
}

byte caldata[68]; // buffer to receive magnetic calibration data
byte calcount=0;

void receiveCalibration() {
  uint16_t crc;
  byte b, i;

  while (Serial.available()) {
    b = Serial.read();
    if (calcount == 0 && b != 117) {
      // first byte must be 117
      return;
    }
    if (calcount == 1 && b != 84) {
      // second byte must be 84
      calcount = 0;
      return;
    }
    // store this byte
    caldata[calcount++] = b;
    if (calcount < 68) {
      // full calibration message is 68 bytes
      return;
    }
    // verify the crc16 check
    crc = 0xFFFF;
    for (i=0; i < 68; i++) {
      crc = crc16_update(crc, caldata[i]);
    }
    if (crc == 0) {
      // data looks good, use it
      float offsets[16];
      memcpy(offsets, caldata+2, 16*4);
      accel_zerog[0] = offsets[0];
      accel_zerog[1] = offsets[1];
      accel_zerog[2] = offsets[2];
      
      gyro_zerorate[0] = offsets[3];
      gyro_zerorate[1] = offsets[4];
      gyro_zerorate[2] = offsets[5];
      
      mag_hardiron[0] = offsets[6];
      mag_hardiron[1] = offsets[7];
      mag_hardiron[2] = offsets[8];

      mag_field = offsets[9];
      
      mag_softiron[0] = offsets[10];
      mag_softiron[1] = offsets[13];
      mag_softiron[2] = offsets[14];
      mag_softiron[3] = offsets[13];
      mag_softiron[4] = offsets[11];
      mag_softiron[5] = offsets[15];
      mag_softiron[6] = offsets[14];
      mag_softiron[7] = offsets[15];
      mag_softiron[8] = offsets[12];


      calcount = 0;
      return;
    }
    // look for the 117,84 in the data, before discarding
    for (i=2; i < 67; i++) {
      if (caldata[i] == 117 && caldata[i+1] == 84) {
        // found possible start within data
        calcount = 68 - i;
        memmove(caldata, caldata + i, calcount);
        return;
      }
    }
    // look for 117 in last byte
    if (caldata[67] == 117) {
      caldata[0] = 117;
      calcount = 1;
    } else {
      calcount = 0;
    }
  }
}


uint16_t crc16_update(uint16_t crc, uint8_t a)
{
  int i;
  crc ^= a;
  for (i = 0; i < 8; i++) {
    if (crc & 1) {
      crc = (crc >> 1) ^ 0xA001;
    } else {
      crc = (crc >> 1);
    }
  }
  return crc;
}

void getCalIMU(float *val) {
   float ax, ay, az;
   float gx, gy, gz;
   float mx, my, mz;

   M5.Imu.getAccel(&ax, &ay, &az);
   M5.Imu.getGyro(&gx, &gy, &gz);
   M5.Imu.getMag(&mx, &my, &mz);
  
   val[0] = -ay;
   val[1] = -ax;
   val[2] = -az;
   
   val[3] = -gy ;
   val[4] = -gx ;
   val[5] = -gz ;
   
   val[6] = mx;  //driver already multiples by 10
   val[7] = my;
   val[8] = mz;
   
   new_mag_data = true;

}


