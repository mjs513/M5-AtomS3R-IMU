// gyro bias estimation
size_t _numSamples = 100;
double _gxbD, _gybD, _gzbD;
float _gxb, _gyb, _gzb;
// accel bias and scale factor estimation
double _axbD, _aybD, _azbD;
float _axmax, _aymax, _azmax;
float _axmin, _aymin, _azmin;
float _axb, _ayb, _azb;
float _axs = 1.0f;
float _ays = 1.0f;
float _azs = 1.0f;
// magnetometer bias and scale factor estimation
uint16_t _maxCounts = 2000;
float _deltaThresh = 0.3f;
uint8_t _coeff = 8;
uint16_t _counter;
float _framedelta, _delta;
float _hxfilt, _hyfilt, _hzfilt;
float _hxmax, _hymax, _hzmax;
float _hxmin, _hymin, _hzmin;
float _hxb, _hyb, _hzb;
float _hxs = 1.0f;
float _hys = 1.0f;
float _hzs = 1.0f;
float _avgs;
   
/* finds bias and scale factor calibration for the accelerometer,
this should be run for each axis in each direction (6 total) to find
the min and max values along each */
int calibrateAccel() {
  // set the range, bandwidth, and srd
/*  if (Imu.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_2G) != 1) {
    return -1;
  
  if (Imu.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_20HZ) != 1) {
    return -2;
  
  if (Imu.ConfigSrd(19) != 1) {
    return -3;
  }
*/
  // take samples and find min / max 
  _axbD = 0;
  _aybD = 0;
  _azbD = 0;
  _axs = 1; _ays = 1;  _azs = 1; 
  _axb = 0; _ayb = 0;  _azb = 0; 
  uint16_t icount = 0;
  float values[9];

  //for (size_t i=0; i < _numSamples; i++) {
  while(icount < _numSamples) {
    readIMU(values);
      if(new_accel_data){
        icount += 1;
        _axbD += (values[0]/_axs + _axb)/((double)_numSamples);
        _aybD += (values[1]/_ays + _ayb)/((double)_numSamples);
        _azbD += (values[2] /_azs + _azb)/((double)_numSamples);
      }
  }

  Serial.printf("%f, %f, %f\n",_axbD, _aybD, _azbD );

  if (_axbD > 9.0f) {
    _axmax = (float)_axbD;
  }
  if (_aybD > 9.0f) {
    _aymax = (float)_aybD;
  }
  if (_azbD > 9.0f) {
    _azmax = (float)_azbD;
  }
  if (_axbD < -9.0f) {
    _axmin = (float)_axbD;
  }
  if (_aybD < -9.0f) {
    _aymin = (float)_aybD;
  }
  if (_azbD < -9.0f) {
    _azmin = (float)_azbD;
  }

  // find bias and scale factor
  if ((abs(_axmin) > 9.0f) && (abs(_axmax) > 9.0f)) {
    _axb = (_axmin + _axmax) / 2.0f;
    _axs = G/((abs(_axmin) + abs(_axmax)) / 2.0f);
  }
  if ((abs(_aymin) > 9.0f) && (abs(_aymax) > 9.0f)) {
    _ayb = (_axmin + _axmax) / 2.0f;
    _ays = G/((abs(_aymin) + abs(_aymax)) / 2.0f);
  }
  if ((abs(_azmin) > 9.0f) && (abs(_azmax) > 9.0f)) {
    _azb = (_azmin + _azmax) / 2.0f;
    _azs = G/((abs(_azmin) + abs(_azmax)) / 2.0f);
  }

  Serial.printf("Max: %f, %f, %f\n",_axmax,_aymax,_azmax );
  Serial.printf("Max: %f, %f, %f\n",_axmin,_aymin,_azmin );
  Serial.printf("%f, %f, %f\n",_axb,_ayb,_azb );
  // set the range, bandwidth, and srd back to what they were
/*  if (setAccelRange(_accelRange) < 0) {
    return -4;
  }
  if (setDlpfBandwidth(_bandwidth) < 0) {
    return -5;
  }
  if (setSrd(_srd) < 0) {
    return -6;
  }
  */
  return 1;  
}


/* estimates the gyro biases */
int calibrateGyro() {
  // set the range, bandwidth, and srd
/*  if (setGyroRange(GYRO_RANGE_250DPS) < 0) {
    return -1;
  }
  if (setDlpfBandwidth(DLPF_BANDWIDTH_20HZ) < 0) {
    return -2;
  }
  if (setSrd(19) < 0) {
    return -3;
  }
*/
  // take samples and find bias
  _gxbD = 0;
  _gybD = 0;
  _gzbD = 0;
  uint16_t icount = 0;
  float values[9];

  //for (size_t i=0; i < _numSamples; i++) {
  while(icount < _numSamples) {
    readIMU(values);
      if(new_gyro_data){
        icount++;
        _gxbD += ((values[3] ) + _gxb)/((double)_numSamples);
        _gybD += ((values[4] ) + _gyb)/((double)_numSamples);
        _gzbD += ((values[5] ) + _gzb)/((double)_numSamples);
      }
  }
  _gxb = (float)_gxbD;
  _gyb = (float)_gybD;
  _gzb = (float)_gzbD;
/*
  // set the range, bandwidth, and srd back to what they were
  if (setGyroRange(_gyroRange) < 0) {
    return -4;
  }
  if (setDlpfBandwidth(_bandwidth) < 0) {
    return -5;
  }
  if (setSrd(_srd) < 0) {
    return -6;
  }
  */
  return 1;
}


/* finds bias and scale factor calibration for the magnetometer,
the sensor should be rotated in a figure 8 motion until complete */
int calibrateMag() {
  // set the srd
/*  if (setSrd(19) < 0) {
    return -1;
  }
*/
  // get a starting set of data
  float values[9];

  readIMU(values);
  _hxmax = values[6];
  _hxmin = values[6];
  _hymax = values[7];
  _hymin = values[7];
  _hzmax = values[8];
  _hzmin = values[8];

  // collect data to find max / min in each channel
  _counter = 0;
  while (_counter < _maxCounts) {
    _delta = 0.0f;
    _framedelta = 0.0f;
    readIMU(values);
      if(new_mag_data){
        _hxfilt = (_hxfilt*((float)_coeff-1)+(values[6]/_hxs+_hxb))/((float)_coeff);
        _hyfilt = (_hyfilt*((float)_coeff-1)+(values[7]/_hys+_hyb))/((float)_coeff);
        _hzfilt = (_hzfilt*((float)_coeff-1)+(values[8]/_hzs+_hzb))/((float)_coeff);
        if (_hxfilt > _hxmax) {
          _delta = _hxfilt - _hxmax;
          _hxmax = _hxfilt;
        }

        if (_delta > _framedelta) {
          _framedelta = _delta;
        }
        if (_hyfilt > _hymax) {
          _delta = _hyfilt - _hymax;
          _hymax = _hyfilt;
        }
        if (_delta > _framedelta) {
          _framedelta = _delta;
        }
        if (_hzfilt > _hzmax) {
          _delta = _hzfilt - _hzmax;
          _hzmax = _hzfilt;
        }
        if (_delta > _framedelta) {
          _framedelta = _delta;
        }
        if (_hxfilt < _hxmin) {
          _delta = abs(_hxfilt - _hxmin);
          _hxmin = _hxfilt;
        }
        if (_delta > _framedelta) {
          _framedelta = _delta;
        }
        if (_hyfilt < _hymin) {
          _delta = abs(_hyfilt - _hymin);
          _hymin = _hyfilt;
        }
        if (_delta > _framedelta) {
          _framedelta = _delta;
        }
        if (_hzfilt < _hzmin) {
          _delta = abs(_hzfilt - _hzmin);
          _hzmin = _hzfilt;
        }
        if (_delta > _framedelta) {
          _framedelta = _delta;
        }
        if (_framedelta > _deltaThresh) {
          _counter = 0;
        } else {
          _counter++;
        }
      }
  }

  // find the magnetometer bias
  _hxb = (_hxmax + _hxmin) / 2.0f;
  _hyb = (_hymax + _hymin) / 2.0f;
  _hzb = (_hzmax + _hzmin) / 2.0f;

  // find the magnetometer scale factor
  _hxs = (_hxmax - _hxmin) / 2.0f;
  _hys = (_hymax - _hymin) / 2.0f;
  _hzs = (_hzmax - _hzmin) / 2.0f;
  _avgs = (_hxs + _hys + _hzs) / 3.0f;
  _hxs = _avgs/_hxs;
  _hys = _avgs/_hys;
  _hzs = _avgs/_hzs;

  // set the srd back to what it was
/*  if (setSrd(_srd) < 0) {
    return -2;
  }
  */
  return 1;
}

//for m5 returns degress/sec
/* returns the gyro bias in the X direction, rad/s */
float getGyroBiasX_rads() {
  return _gxb;
}

/* returns the gyro bias in the Y direction, rad/s */
float getGyroBiasY_rads() {
  return _gyb;
}

/* returns the gyro bias in the Z direction, rad/s */
float getGyroBiasZ_rads() {
  return _gzb;
}

//For m5 actually returns corrects in g's
/* returns the accelerometer bias in the X direction, m/s/s */
float getAccelBiasX_mss() {
  return _axb;
}

/* returns the accelerometer scale factor in the X direction */
float getAccelScaleFactorX() {
  return _axs;
}

/* returns the accelerometer bias in the Y direction, m/s/s */
float getAccelBiasY_mss() {
  return _ayb;
}

/* returns the accelerometer scale factor in the Y direction */
float getAccelScaleFactorY() {
  return _ays;
}

/* returns the accelerometer bias in the Z direction, m/s/s */
float getAccelBiasZ_mss() {
  return _azb;
}

/* returns the accelerometer scale factor in the Z direction */
float getAccelScaleFactorZ() {
  return _azs;
}


/* returns the magnetometer bias in the X direction, uT */
float getMagBiasX_uT() {
  return _hxb;
}

/* returns the magnetometer scale factor in the X direction */
float getMagScaleFactorX() {
  return _hxs;
}

/* returns the magnetometer bias in the Y direction, uT */
float getMagBiasY_uT() {
  return _hyb;
}

/* returns the magnetometer scale factor in the Y direction */
float getMagScaleFactorY() {
  return _hys;
}

/* returns the magnetometer bias in the Z direction, uT */
float getMagBiasZ_uT() {
  return _hzb;
}

/* returns the magnetometer scale factor in the Z direction */
float getMagScaleFactorZ() {
  return _hzs;
}
