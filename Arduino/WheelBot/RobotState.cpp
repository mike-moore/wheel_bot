#include "RobotState.h"

RobotState::RobotState() 
 :
 effectors(),
 SensedHeading(0.0),
 SensedDistance(0.0),
 ActiveWayPoint(),
 WayPointQueue(),
 HeadingError(0.0),
 HeadingErrorTol(2.0),
 DistanceError(0.0),
 DistanceErrorTol(0.25),
 ControlSignal(0.0),
 ClosedLoopControl(false),
 HeadingReached(true),
 DistanceReached(true)
{
	strncpy(ActiveWayPoint.Name,"StartWayPoint", 15);
	ActiveWayPoint.Heading = 0.0;
	ActiveWayPoint.Distance = 0.0;
}

