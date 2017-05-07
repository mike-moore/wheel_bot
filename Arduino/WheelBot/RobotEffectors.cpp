#include "RobotEffectors.h"

RobotEffectors::RobotEffectors() 
 :
 leftMotor(mtrL_motorPinA1, mtrL_motorPinB1, 
                                  mtrL_motorPwmPin, mtrL_encoderCount, mtrL_speed),
 rightMotor(mtrR_motorPinA1, mtrR_motorPinB1, 
                                  mtrR_motorPwmPin, mtrR_encoderCount, mtrR_speed)
{
}

