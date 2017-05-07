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

///////////////////////////////////////////////////////////////
/// @class Control
/// @ingroup WheelBot
/// @brief TODO
///////////////////////////////////////////////////////////////
class Control { 
 public:
    Control(RobotState& state) : State(state), _velocityCmd(0.0), 
            _positionCmd(0.0), _Kp(5.0), _Kd(0.5), _lastMilliPrint(0),
            Mode(IDLE), _testDriveState(FWD_POS_TEST), _numSecondsInTest(0.0),
            _firstPass(true) {};
    ~Control(){};

    void Execute();

 private:

    bool _headingError();
    void _checkForDistanceControl();
    bool _distanceError();
    void _printHeadingDebug();
    void _printDistanceDebug();
    void _performRotationControl();
    void _testDrive();
    RobotState& State;
    float _velocityCmd;
    float _positionCmd;
    float _Kp;
    float _Kd;
    unsigned long _lastMilliPrint;
    typedef enum _ControlMode
    {
      IDLE            = 0,
      ROTATIONAL_CTRL = 1,
      DRIVING_FWD     = 2,
      DRIVING_BWD     = 3,
      TEST_DRIVE      = 4
    }ControlMode;
    ControlMode Mode;
	// Moding used for a test drive
	enum TestDriveRoute
	{
	    FWD_POS_TEST = 0,
	    BWD_POS_TEST = 1,
	    SPEED_UP_VEL_TEST = 2,
      SLOW_DOWN_VEL_TEST = 3     
	};
    TestDriveRoute _testDriveState;
    float _numSecondsInTest;
    bool _firstPass;
};
  

#endif
