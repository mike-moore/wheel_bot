#include "Magnetometer.h"

#define MAGN_ADDRESS ((int) 0x1E) // 0x1E = 0x3C / 2

// Arduino backward compatibility macros
#if ARDUINO >= 100
  #define WIRE_SEND(b) Wire.write((byte) b)
  #define WIRE_RECEIVE() Wire.read()
#else
  #define WIRE_SEND(b) Wire.send(b)
  #define WIRE_RECEIVE() Wire.receive()
#endif

void Magnetometer::Init()
{
  //Init Sensors
  delay(50); //Give sensors enough time to start
  _i2c_init();
  _magn_init();
  delay(20);
}

double Magnetometer::ReadHeading()
{
    // Update sensor readings
    _read_mag();
    // Apply sensor calibration
    _compensate_sensor_errors();
    // Run DCM algorithm
    _compass_heading(); // Calculate magnetic heading
    return MAG_Heading;
}

void Magnetometer::_i2c_init()
{
  Wire.begin();
}

void Magnetometer::_magn_init()
{
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x02);
  WIRE_SEND(0x00);  // Set continuous mode (default 10Hz)
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x00);
  WIRE_SEND(0b00011000);  // Set 50Hz
  Wire.endTransmission();
  delay(5);
}

void Magnetometer::_read_mag()
{
  int i = 0;
  byte buff[6];

  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x03);  // Send address to read from
  Wire.endTransmission();

  Wire.beginTransmission(MAGN_ADDRESS);
  Wire.requestFrom(MAGN_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  {
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  
  Wire.endTransmission();

  if (i == 6) // All bytes received?
  {
    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
    magnetom[0] = (int16_t)((((uint16_t) buff[0]) << 8) | buff[1]);         // X axis (internal sensor x axis)
    magnetom[1] = -1 * (int16_t)(((((uint16_t) buff[4]) << 8) | buff[5]));  // Y axis (internal sensor -y axis)
    magnetom[2] = -1 * (int16_t)(((((uint16_t) buff[2]) << 8) | buff[3]));  // Z axis (internal sensor -z axis)
  }
}

void Magnetometer::_compensate_sensor_errors()
{
  magnetom[0] = (magnetom[0] - mag_bias[0]) * mag_scale[0];
  magnetom[1] = (magnetom[1] - mag_bias[1]) * mag_scale[1];
  magnetom[2] = (magnetom[2] - mag_bias[2]) * mag_scale[2];
}

void Magnetometer::_compass_heading()
{
  // Magnetic Heading
  MAG_Heading = (atan2(magnetom[1], magnetom[0])*180.0/3.14159)+180.0;
  MAG_Heading = constrain(MAG_Heading, 0.0, 360.0);
}
