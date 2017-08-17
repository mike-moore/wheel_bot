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
#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "RobotState.h"

#define SIZE_HEADING_BUFFER 10
#define SIZE_ERROR_BUFFER 10

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
    float getFilteredHeading(float sensedHeading);
    float sensedHeadingBuffer[SIZE_HEADING_BUFFER];

 private:
    void avgError();
    RobotState& State;
    float _errorBuffer[SIZE_ERROR_BUFFER];
};
  

#endif
