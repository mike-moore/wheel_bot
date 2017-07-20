#include "RobotState.h"

RobotState::RobotState() 
 :
 effectors(),
 FilteredSensedHeading(0.0),
 SensedDistance(0.0),
 ActiveWayPoint(),
 WayPointQueue(),
 HeadingError(0.0),
 HeadingErrorTol(4.0),
 DistanceError(0.0),
 DistanceErrorTol(0.25),
 ControlSignal(0.0),
 HeadingReached(true),
 DistanceReached(true)
{
	strncpy(ActiveWayPoint.Name,"StartWayPoint", 15);
	ActiveWayPoint.Heading = 0.0;
	ActiveWayPoint.Distance = 0.0;
}

