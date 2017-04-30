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
    Control(RobotState& state) : State(state), ControlWaitCycles(0), firstPass(true) {};
    ~Control(){};

    void Execute();

 private:
    void testDrive();
    RobotState& State;
    uint32_t ControlWaitCycles;
	// Moding used for a test drive
	enum TestDriveRoute
	{
	    FWD_POS_TEST = 0,
	    BWD_POS_TEST = 1,
	    FWD_VEL_TEST = 2,
	    ZERO_VEL_TEST = 3,
	    BWD_VEL_TEST = 4
	};
    TestDriveRoute TestDriveState;
    bool firstPass;
};
  

#endif
