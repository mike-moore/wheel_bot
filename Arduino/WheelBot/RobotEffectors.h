///////////////////////////////////////////////////////////////
///  This file defines a class that is used to implement
///  the TODO.
///
/// @author
///         $Author: Mike Moore $
///
/// Contact: mike.moore@so.engineering
///
/// Created on: March 23 2017
///
///////////////////////////////////////////////////////////////
#ifndef ROBOTEFFECTORS_H
#define ROBOTEFFECTORS_H

#include "Spg30MotorDriver.h"

// Left and Right Motor Pins
enum RightMotorPinAssignments {
    mtrR_encoderPinA = 7,
    mtrR_encoderPinB = 6,
    mtrR_motorPwmPin = 9,
    mtrR_motorPinA1  = 11,
    mtrR_motorPinB1  = 10
};

enum LeftMotorPinAssignments {
    mtrL_encoderPinA = 4,
    mtrL_encoderPinB = 5,
    mtrL_motorPwmPin = 8,
    mtrL_motorPinA1  = 12,
    mtrL_motorPinB1  = 13
};

extern const long cycleTimeMillis;
extern volatile long int mtrR_encoderCount;
extern volatile long int mtrL_encoderCount;
extern int mtrR_speed;
extern int mtrL_speed;
///////////////////////////////////////////////////////////////
/// @class RobotEffectors
/// @ingroup WheelBot
/// @brief TODO
///////////////////////////////////////////////////////////////
class RobotEffectors { 
 public:
  RobotEffectors();
  ~RobotEffectors(){};

  /// - Motors
  Spg30MotorDriver leftMotor;
  Spg30MotorDriver rightMotor;

 private:
 	/// - None yet
};

#endif
