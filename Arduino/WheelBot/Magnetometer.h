#include <Arduino.h>
#include <Wire.h>

#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)

#ifndef MAGNETOMETER_HH
#define MAGNETOMETER_HH

const float mag_bias[3] = {-24.5, 328.5, 99.5};
const float mag_scale[3] = {0.890,1.02,1.11};

class Magnetometer
{
  public:
    Magnetometer(){};
    void Init();
    double ReadHeading();

  private:
    void _i2c_init();
    void _magn_init();
    void _read_mag();
    void _compensate_sensor_errors();
    void _compass_heading();
    float magnetom[3];
    float MAG_Heading;  
};

#endif
