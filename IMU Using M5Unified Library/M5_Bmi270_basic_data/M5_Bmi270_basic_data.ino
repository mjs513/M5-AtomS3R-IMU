#include <M5Unified.h>


float raw_values[9];
bool new_mag_data = false;
const float deg2rad = PI/180.0;

void getCalIMU(float *val) {
   float ax, ay, az;
   float axo, ayo, azo;
   float gx, gy, gz;
   float gxo, gyo, gzo;
   float mxo, myo, mzo;
   float mx, my, mz;

   M5.Imu.getAccel(&ax, &ay, &az);
   M5.Imu.getGyro(&gx, &gy, &gz);
   M5.Imu.getMag(&mx, &my, &mz);
   
   // Match axis orientation from example
   // reverse axis alignment
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
}

void setup() {
   auto cfg = M5.config();
   M5.begin(cfg);
   M5.Imu.begin();
   Serial.begin(115200);
}

void loop() {
   getCalIMU(raw_values);
// float value[3] = { 8.0f / 32768.0f, 2000.0f / 32768.0f, 10.0f * 4912.0f / 32768.0f };
   if(new_mag_data) {
       Serial.print("Raw:");
       Serial.print((raw_values[0]), 4 ); Serial.print(',');
       Serial.print((raw_values[1]), 4 ); Serial.print(',');
       Serial.print((raw_values[2] ), 4 ); Serial.print(',');
       Serial.print((raw_values[3] ), 4 ); Serial.print(',');
       Serial.print((raw_values[4] ), 4 ); Serial.print(',');
       Serial.print((raw_values[5] ), 4 ); Serial.print(',');
       Serial.print((raw_values[6] ), 4 ); Serial.print(',');
       Serial.print((raw_values[7] ), 4 ); Serial.print(',');
       Serial.print((raw_values[8] ), 4 ); Serial.println();
       
       new_mag_data = false;
   }
   
   delay(10);
}
