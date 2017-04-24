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
#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "RobotState.h"

///////////////////////////////////////////////////////////////
/// @class Navigation
/// @ingroup WheelBot
/// @brief TODO
///////////////////////////////////////////////////////////////
class Navigation { 
 public:
    Navigation(RobotState& state) : State(state) {};
    ~Navigation(){};

    void InitSensors();
    void Execute();

 private:
    RobotState& State;
};
  

#endif
