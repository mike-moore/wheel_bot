#include <Arduino.h>
#include <Wire.h>

#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 20  // in milliseconds

#define CALIBRATION__MAGN_USE_EXTENDED true

#ifndef MAGNETOMETER_HH
#define MAGNETOMETER_HH

const float magn_ellipsoid_center[3] = {168.647, 210.785, 108.989};
const float magn_ellipsoid_transform[3][3] = {{0.806619, -0.0115270, -0.0105417},
                                              {-0.0115270, 0.845341, 0.00713515},
                                              {-0.0105417, 0.00713515, 0.999034}};
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
    unsigned long timestamp;
    float magnetom[3];
    float MAG_Heading;
    
};

#endif
