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
#ifndef GUIDANCE_H
#define GUIDANCE_H

#include "RobotState.h"

///////////////////////////////////////////////////////////////
/// @class Guidance
/// @ingroup WheelBot
/// @brief TODO
///////////////////////////////////////////////////////////////
class Guidance { 
 public:
    Guidance(RobotState& state) : State(state), Mode(IDLE) {};
    ~Guidance(){};

    void Execute();
    inline int getMode() { return Mode; }

 private:
    RobotState& State;
    typedef enum _GuidanceMode
    {
      IDLE = 0,
      TRACKING = 1,
      MAPPING = 2
    }GuidanceMode;
    GuidanceMode Mode;
};
  

#endif
