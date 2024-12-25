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
    M5.Display.setRotation(1);
    M5.Display.setTextSize(3);

  //setup moving average
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
    M5.Display.clear();
    M5.Display.setCursor(10, 10);
    M5.Display.printf("%.1f", heading);
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
