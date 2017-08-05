///////////////////////////////////////////////////////////////
///  This file defines a class that is used to implement
///  the TODO.
///
/// @author
///         $Author: Mike Moore $
///
/// Created on: March 23 2017
///
///////////////////////////////////////////////////////////////
#ifndef CONTROL_H
#define CONTROL_H

#include "RobotState.h"

///////////////////////////////////////////////////////////////
/// @class Control
/// @ingroup WheelBot
/// @brief TODO
///////////////////////////////////////////////////////////////
class Control { 
 public:
    Control(RobotState& state) : State(state), _velocityCmd(0.0), 
            _positionCmd(0.0), MotorRotDegPerFt(330), MotorRotDegPerDegHeading(3.6), _lastMilliPrint(0),
            Mode(IDLE), _testDriveState(DRIVE_FWD_OL), _firstPass(true), _prevError(0.0), _errorAccum(0.0), _Kp_Left(0.155), _Kp_Right(0.165),
            _Ki_Right(0.0001), _Ki_Left(0.0001), _Kd_Right(0.0001), _Kd_Left(0.0001), _TurnRightCount(0), _cmdHeading(0.0) {};
 
    ~Control(){};

    void Execute();

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
    RobotState& State;
    float _velocityCmd;
    float _positionCmd;
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
    float _prevError;
    float _errorAccum;
    float _Kp_Left;
    float _Kp_Right;
    float _Ki_Right;
    float _Ki_Left;
    float _Kd_Right;
    float _Kd_Left;
    uint16_t _TurnRightCount;
    float _cmdHeading;
};
  

#endif
