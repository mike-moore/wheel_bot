#include "Navigation.h"

void Navigation::InitSensors() {
    //State.sensors.magnetometer.Init();
}

void Navigation::Execute() {
    //State.SensedHeading = State.sensors.magnetometer.ReadHeading();
    State.SensedHeading  = 0.0;
    State.SensedDistance = 0.0;
    //Serial.print("Sensed heading : ");
    //Serial.println(State.SensedHeading);
}

