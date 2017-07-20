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
#ifndef CONTROL_H
#define CONTROL_H

#include "RobotState.h"

#define SIZE_ERROR_BUFFER 10

///////////////////////////////////////////////////////////////
/// @class Control
/// @ingroup WheelBot
/// @brief TODO
///////////////////////////////////////////////////////////////
class Control { 
 public:
    Control(RobotState& state) : State(state), _velocityCmd(0.0), 
            _positionCmd(0.0), _Kp(5.0), _Kd(0.5), MotorRotDegPerFt(330), MotorRotDegPerDegHeading(3.6), _lastMilliPrint(0),
            Mode(IDLE), _testDriveState(HEADING_CONTROL_TEST), _firstPass(true), errorRate(0.0), 
            velocityRight(0.0), velocityLeft(0.0), prev_error(0.0), _Kp_Left(0.0), _Kp_Right(0.0),_Kd_Right(0.0), _Kd_Left(0.0){};

    ~Control(){};

    void Execute();
    float getFilteredError(float error);
    float errorBuffer[SIZE_ERROR_BUFFER];

 private:
    bool _headingError();
    void _checkForDistanceControl();
    void _checkForHeadingControl();
    void _performHeadingControl();
    bool _distanceError();
    void _printHeadingDebug();
    void _printDistanceDebug();
    void _performRotationControl();
    void _testDrive();
    float _pdHeadingControl();
    RobotState& State;
    float _velocityCmd;
    float _positionCmd;
    float _Kp;
    float _Kd;
    uint16_t MotorRotDegPerFt;
    float MotorRotDegPerDegHeading;
    unsigned long _lastMilliPrint;
    typedef enum _ControlMode
    {
      IDLE                = 0,
      ROTATIONAL_CTRL     = 1,
      TRANSLATIONAL_CTRL  = 2,
      TEST_DRIVE          = 3
    }ControlMode;
    ControlMode Mode;
    enum TestDriveRoute
    {
        DRIVE_FWD_OL       = 0,
        TURN_RIGHT_OL      = 1,
        DRIVE_FWD_CL       = 2,
        TURN_RIGHT_CL      = 3  
    } _testDriveState;
    bool _firstPass;
    float errorRate;
    float velocityRight;
    float velocityLeft;
    float prev_error;
    float _Kp_Left;
    float _Kp_Right;
    float _Kd_Right;
    float _Kd_Left;
};
  

#endif
