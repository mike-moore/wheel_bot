#include "Guidance.h"

void Guidance::Execute() {
    switch(Mode)
    {
        case IDLE:
            /// - Zero the errors sent to control
            State.HeadingError = 0.0;
            State.DistanceError = 0.0;
            if (State.WayPointQueue.isEmpty()){
                return;
            }else{
                /// - Pop the waypoint off the queue and set to active way point.
                WayPoint way_point;
                way_point = State.WayPointQueue.pop();
                /// - Set the active way point and move to trackind mode.
                strncpy(State.ActiveWayPoint.Name, way_point.Name, 15);
                State.ActiveWayPoint.Heading = way_point.Heading; 
                State.ActiveWayPoint.Distance = way_point.Distance; 
                Serial.print("Tracking to new waypoint : ");
                Serial.println(State.ActiveWayPoint.Name);
                /// - Compute the errors for control
                State.HeadingError =  State.ActiveWayPoint.Heading; //- State.FilteredSensedHeading;
                State.DistanceError = State.ActiveWayPoint.Distance; //- State.SensedDistance;
                Mode = TRACKING;
            }
        break;

        case TRACKING:
            if (State.HeadingReached && State.DistanceReached){
                Mode = IDLE;
            }
        break;

        case MAPPING:
             /// - To be implemented
             Mode = IDLE;
        break;

        default:
             /// - invalid state - go back to IDLE
             Mode = IDLE;
        break;
    }
    return;
}
