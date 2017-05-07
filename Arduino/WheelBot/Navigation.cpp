#include "Navigation.h"

void Navigation::InitSensors() {
    State.sensors.magnetometer.Init();
}

void Navigation::Execute() {
    //State.SensedHeading = State.sensors.magnetometer.ReadHeading();
    State.SensedDistance = 2.0;
    //Serial.print("Sensed heading : ");
    //Serial.println(State.SensedHeading);
}

