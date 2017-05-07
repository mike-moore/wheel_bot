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
#ifndef ROBOTSENSORS_H
#define ROBOTSENSORS_H

#include "Magnetometer.h"

///////////////////////////////////////////////////////////////
/// @class RobotSensors
/// @ingroup WheelBot
/// @brief TODO
///////////////////////////////////////////////////////////////
class RobotSensors { 
 public:
  RobotSensors();
  ~RobotSensors(){};

  /// - Sensors
  Magnetometer magnetometer;

 private:
 	/// - None yet
};

#endif
