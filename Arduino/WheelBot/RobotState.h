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
#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include "RobotSensors.h"
#include "RobotEffectors.h"
#include "comm_packet.pb.h"
#include "QueueList.h"

///////////////////////////////////////////////////////////////
/// @class RobotState
/// @ingroup WheelBot
/// @brief TODO
///////////////////////////////////////////////////////////////
class RobotState { 
 public:
  RobotState();
  ~RobotState(){};

  /// - Robotic sensors - magnetometer
  RobotSensors sensors;
  /// - Robotic effectors - motor driver(s)
  RobotEffectors effectors;

  /// - Sensor readings
  float SensedHeading;
  float SensedDistance;

  /// - Robot guidance
  WayPoint ActiveWayPoint;
  QueueList <WayPoint> WayPointQueue;
  float HeadingError;
  float HeadingErrorTol;
  float DistanceError;
  float DistanceErrorTol;

  /// - Robot control
  float ControlSignal;
  bool DoTestDrive;
  bool TargetReached;

  /// - Telemetry

 private:
 	/// - None yet
};

#endif
