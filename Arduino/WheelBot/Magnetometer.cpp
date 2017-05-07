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
    // Time to read the sensors again?
    if((millis() - timestamp) >= OUTPUT__DATA_INTERVAL)
    {
      timestamp = millis();
      // Update sensor readings
      _read_mag();
      // Apply sensor calibration
      _compensate_sensor_errors();
      // Run DCM algorithm
      _compass_heading(); // Calculate magnetic heading
    }
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

void _mat_vec_mult(const float a[3][3], const float b[3], float out[3])
{
  for(int x = 0; x < 3; x++)
  {
    out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
  }
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
      magnetom[0] = (((int) buff[0]) << 8) | buff[1];         // X axis (internal sensor x axis)
      magnetom[1] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // Y axis (internal sensor -y axis)
      magnetom[2] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // Z axis (internal sensor -z axis)
    }
}

void Magnetometer::_compensate_sensor_errors()
{
  float magnetom_tmp[3];
  // Compensate magnetometer error
  #if CALIBRATION__MAGN_USE_EXTENDED == true
      for (int i = 0; i < 3; i++)
        magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
      _mat_vec_mult(magn_ellipsoid_transform, magnetom_tmp, magnetom);
  #else
      magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
      magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
      magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
  #endif
}

void Magnetometer::_compass_heading()
{
    float mag_x;
    float mag_y;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;

    cos_roll = cos(0.0);
    sin_roll = sin(0.0);
    cos_pitch = cos(0.0);
    sin_pitch = sin(0.0);

    // Tilt compensated magnetic field X
    mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch;
    // Tilt compensated magnetic field Y
    mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
    // Magnetic Heading
    MAG_Heading = atan2(-mag_y, mag_x);
}
