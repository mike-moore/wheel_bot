///////////////////////////////////////////////////////////////
///  This file defines a class that is used to implement
///  a motor control driver for the SPG30E-XXXk DC motor
///  for the Arduino platform.
///
/// @author
///         $Author: Mike Moore $
///
/// Contact: michael.moore@nasa.gov
///
/// Created on: February 5 2017
///
///////////////////////////////////////////////////////////////
#ifndef SPG30MOTORDRIVER_H
#define SPG30MOTORDRIVER_H

#include <Arduino.h>

#define MTR_DEADBAND_LOW 125
static const int _pwmLookup[30] =
{
    MTR_DEADBAND_LOW, MTR_DEADBAND_LOW, MTR_DEADBAND_LOW, MTR_DEADBAND_LOW, MTR_DEADBAND_LOW,
    MTR_DEADBAND_LOW, MTR_DEADBAND_LOW, MTR_DEADBAND_LOW, MTR_DEADBAND_LOW, MTR_DEADBAND_LOW,
    MTR_DEADBAND_LOW, MTR_DEADBAND_LOW, MTR_DEADBAND_LOW, MTR_DEADBAND_LOW, MTR_DEADBAND_LOW,
    130, 135, 140, 145, 150,
    160, 165, 170, 180, 190,
    200, 210, 220, 230, 240,
}; 

class Spg30MotorDriver { 
 public:
  /// @brief Construct the driver by providing all Arduino pin assignments
  Spg30MotorDriver(uint_least8_t motorPinA1, uint_least8_t motorPinB1, 
                   uint_least8_t pwmPin, volatile long int& encoderCount,
  	               int & motorSpeed);

  /// @brief Defines this motor driver's modes of operation.
  enum ControlModes
  {
      POSITION = 0,
      VELOCITY = 1,
      IDLE     = 2
  };
  /// @brief User settable mode. Defaults to IDLE. The user
  ///        should set this before attempting a PositionCmd
  ///        or VelocityCmd.
  ControlModes ControlMode;
  /// @brief Defines the different speeds that can be used
  ///        for position control.
  enum MotorSpeeds
  {
      STOP         = 0,
      VEL_LOW      = 100,
      VEL_MEDIUM   = 200,
      VEL_HIGH     = 255
  };
  /// @brief The motor speed used for position control.
  /// @note  This is irrelevant for velocity control.
  MotorSpeeds MotorSpeed;
  /// @brief Driver's primary update routine. This runs the
  ///        driver's internal state machine.
  void run();
  /// @brief Motor control command functions
  void FwdPositionCmd(uint16_t positionCmd);
  void BwdPositionCmd(uint16_t positionCmd);
  void VelocityCmd(int velocityCmd);
  /// @brief Functions the user can call to see if their commanded
  ///        position and velocity have been reached.
  bool ReachedPosition();
 private:
  void _pidControl();
  void _updatePid();
  void _positionControl();
  void _motorBackward();
  void _motorForward();
  void _motorBrake();
  void _printMotorInfo();
  void _cmdPosition(uint16_t positionCmd);
  uint_least8_t _motorPinA1;
  uint_least8_t _motorPinB1;
  uint_least8_t _pwmPin;
  volatile long & _encoderCount;
  int & _measuredSpeed;
  long _countInit;
  long _tickNumber;
  int _velocityCmd;
  int16_t _positionCmd;
  bool _driveForward; 
  bool _driveBackward; 
  uint_least8_t _pwmCmd;
  int _errorAccum;
  unsigned long _lastMilliPrint;
  float _Kp;
  float _Ki;
  bool _motorIsRunning; 
  bool _positionReached;
};

inline bool Spg30MotorDriver::ReachedPosition(){
	return _positionReached;
}

inline void Spg30MotorDriver::VelocityCmd(int velocityCmd){
  _velocityCmd = constrain(velocityCmd, -30, 30);  
  ControlMode = VELOCITY;
}

inline void Spg30MotorDriver::FwdPositionCmd(uint16_t positionCmd){
  _driveForward = true;
  _driveBackward = false;
  _cmdPosition(positionCmd);
  Serial.print("Accepting fwd position command (deg) : ");
  Serial.println(positionCmd);
}

inline void Spg30MotorDriver::BwdPositionCmd(uint16_t positionCmd){
  _driveBackward = true;
  _driveForward = false;
  _cmdPosition(positionCmd);
  Serial.print("Accepting bwd position command (deg) : ");
  Serial.println(positionCmd);
}

// - Logic here checks that we have a non-zero position command,
//   AND that we're not already trying to reach a position, AND
//   that we haven't received the same command we're currently executing.
inline void Spg30MotorDriver::_cmdPosition(uint16_t positionCmd){
  if (positionCmd!=0 && _positionReached && positionCmd!=_positionCmd){
    _positionCmd = positionCmd;
    _tickNumber = positionCmd;
    _countInit = _encoderCount;
    _positionReached = false;
    ControlMode = POSITION;
  }
}

#endif
