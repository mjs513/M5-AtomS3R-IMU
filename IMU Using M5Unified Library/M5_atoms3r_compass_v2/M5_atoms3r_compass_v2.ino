#include <M5Unified.h>
#include "elapsedMillis.h"
#include "Arduino.h"
#include "Fusion.h"

//Moving average class - thank you copilot :)
#include <vector>
class MovingAverage {
public:
    MovingAverage(size_t windowSize) : windowSize(windowSize), window(windowSize, 0.0), index(0), sum(0.0) {}

    double next(double value) {
        sum -= window[index];
        window[index] = value;
        sum += value;
        index = (index + 1) % windowSize;
        return sum / windowSize;
    }

private:
    size_t windowSize;
    std::vector<double> window;
    size_t index;
    double sum;
};
MovingAverage ma(20); // Window size of 10


//Setup Fusion calibration and vectors
FusionOffset offset;
FusionAhrs ahrs;

// Define calibration (replace with actual calibration data if available)
  const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
  FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};

  const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  const FusionVector accelerometerSensitivity = {1.001222, 1.003922, 0.997322};
  const FusionVector accelerometerOffset = {0.046889, 0.046889, 0.040066};

  const FusionMatrix softIronMatrix = {0.9603,0.0076,0.0104,0.0076,0.9566,-0.0132,0.0104,-0.0132,1.0890};
  const FusionVector hardIronOffset = {12.416,19.487,-8.849};

//constants
uint32_t lastUpdate, now1;
double dt, heading;
float fHeading = 0;
float val[10];
// gyro bias estimation
float gyrox_off, gyroy_off, gyroz_off;
uint8_t new_gyro_data = 0;

//Use elapased millis to determing when to run fusion (ensures 100hz update) and dump to screen
elapsedMillis em;
elapsedMillis emc;

void setup() {
  //required config for M5AtomS3r
   auto cfg = M5.config();
   M5.begin(cfg);
   M5.Imu.begin();

  /* Serial to display data */
  while(!Serial && millis() < 5000) {}
  Serial.begin(115200);
  
  //setup IMU
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

  //configure Fusion Settings......
  float SAMPLE_RATE = 0.01f;  //gyro sample rate
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

  //run gyro calibration for fusion
  gyroCalibration();

  //configure display
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextColor(GREEN , BLACK);
  M5.Display.setTextSize(2);
  M5.Display.clearDisplay();
  M5.Display.setRotation(3);

  em = 0;
  emc = 0;
}

void loop() {
  if(em > 10) {
    getFusion();
    heading = ma.next(fHeading);
    //Serial.println(heading);
    em = 0;
  }

  if(emc > 250) {
    Display_Compass(heading);
    emc = 0;
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
   
   new_gyro_data = true;

  if(new_gyro_data) {
    now1= micros(); // This is when the data reported READY
    //Serial.printf(" %f, %f, %f, %f, %f, %f, %f, %f, %f\n", val[0], val[1], 
    //            val[2], val[3], val[4], val[5], val[6], val[7], val[8]);
  }
}


void getFusion() {
  getCalIMU();

  if(new_gyro_data) {
    dt = ((now1 - lastUpdate) * 0.000001);
    lastUpdate = now1;
    new_gyro_data = 0;

    // Acquire latest sensor data
    FusionVector gyroscope = { val[3], val[4], val[5] }; // replace this with actual gyroscope data in degrees/s
    FusionVector accelerometer = { val[0], val[1], val[2] }; // replace this with actual accelerometer data in g
    FusionVector magnetometer = {  val[6], val[7], val[8] }; // replace this with actual magnetometer data in arbitrary units

    // Apply calibration
    //gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
    magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, dt);
    fHeading = FusionCompassCalculateHeading(FusionConventionNed, accelerometer, magnetometer);
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
  This software, the ideas and concepts is Copyright (c) David Bird 2021 and beyond.
  All rights to this software are reserved.
  It is prohibited to redistribute or reproduce of any part or all of the software contents in any form other than the following:
  1. You may print or download to a local hard disk extracts for your personal and non-commercial use only.
  2. You may copy the content to individual third parties for their personal use, but only if you acknowledge the author David Bird as the source of the material.
  3. You may not, except with my express written permission, distribute or commercially exploit the content.
  4. You may not transmit it or store it in any other website or other form of electronic retrieval system for commercial purposes.
  5. You MUST include all of this copyright and permission notice ('as annotated') and this shall be included in all copies
  or substantial portions of the software and where the software use is visible to an end-user.

  THE SOFTWARE IS PROVIDED "AS IS" FOR PRIVATE USE ONLY, IT IS NOT FOR COMMERCIAL USE IN WHOLE OR PART OR CONCEPT.

  FOR PERSONAL USE IT IS SUPPLIED WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.

  IN NO EVENT SHALL THE AUTHOR OR COPYRIGHT HOLDER BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/*
  compass code : https://github.com/G6EJD/ESP-GPS-Compass/blob/master/ESP_GPS_Compass_01.ino#L104
*/
const int centreX  = 64; // Location of the compass display on screen
const int centreY  = 64;
const int diameter = 64; // Size of the compass
int       dx = centreX, dy = centreY;
int       last_dx = centreX, last_dy = centreY - diameter * 0.85;

void Display_Compass(float dBearing) {
  int dxo, dyo, dxi, dyi;
  M5.Display.clearDisplay();
  M5.Display.setCursor(0, 0);
  M5.Display.drawCircle(centreX, centreY, diameter, TFT_WHITE); // Draw compass circle
  for (float i = 0; i < 360; i = i + 22.5) {
    dxo = diameter * cos((i - 90) * 3.14 / 180);
    dyo = diameter * sin((i - 90) * 3.14 / 180);
    dxi = dxo * 0.9;
    dyi = dyo * 0.9;
    M5.Display.drawLine(dxo + centreX, dyo + centreY, dxi + centreX, dyi + centreY, WHITE);
  }
  PrintText((centreX - 5), (centreY - diameter + 15), "N", TFT_GREEN, 2);
  PrintText((centreX - 5), (centreY + diameter - 25) , "S", TFT_GREEN, 2);
  PrintText((centreX + diameter - 25),  (centreY - 5), "E", TFT_GREEN, 2);
  PrintText((centreX - diameter + 15), (centreY - 5), "W", TFT_GREEN, 2);  //was W
  dx = (0.85 * diameter * cos((dBearing - 90) * 3.14 / 180)) + centreX; // calculate X position
  dy = (0.85 * diameter * sin((dBearing - 90) * 3.14 / 180)) + centreY; // calculate Y position
  draw_arrow(last_dx, last_dy, centreX, centreY, 5, 5, TFT_BLUE);   // Erase last arrow
  draw_arrow(dx, dy, centreX, centreY, 5, 5, TFT_YELLOW);           // Draw arrow in new position
  last_dx = dx;
  last_dy = dy;
}
//#####################################################################
void draw_arrow(int x2, int y2, int x1, int y1, int alength, int awidth, int colour) {
  float distance;
  int dx, dy, x2o, y2o, x3, y3, x4, y4, k;
  distance = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
  dx = x2 + (x1 - x2) * alength / distance;
  dy = y2 + (y1 - y2) * alength / distance;
  k = awidth / alength;
  x2o = x2 - dx;
  y2o = dy - y2;
  x3 = y2o * k + dx;
  y3 = x2o * k + dy;
  x4 = dx - y2o * k;
  y4 = dy - x2o * k;
  M5.Display.drawLine(x1, y1, x2, y2, colour);
  M5.Display.drawLine(x1, y1, dx, dy, colour);
  M5.Display.drawLine(x3, y3, x4, y4, colour);
  M5.Display.drawLine(x3, y3, x2, y2, colour);
  M5.Display.drawLine(x2, y2, x4, y4, colour);
}

//#####################################################################
void PrintText(int x, int y, String text, int colour, byte text_size) {
  M5.Display.setCursor(x, y);
  M5.Display.setTextColor(colour);
  M5.Display.setTextSize(text_size);
  M5.Display.print(text);
  M5.Display.setTextColor(TFT_YELLOW); // Default colour
  M5.Display.setTextSize(2);       // Default Text Size
}