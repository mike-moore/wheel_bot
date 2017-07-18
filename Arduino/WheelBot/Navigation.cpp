#include "Navigation.h"

void Navigation::InitSensors() {
    State.sensors.magnetometer.Init();
    for (int i = 0; i < SIZE_HEADING_BUFFER; i++){
        sensedHeadingBuffer[i] = 0.0;
    }    
}

void Navigation::Execute() {
    float sensedHeading = State.sensors.magnetometer.ReadHeading();
    State.FilteredSensedHeading = getFilteredHeading(sensedHeading);
    //State.SensedHeading  = 0.0;
    State.SensedDistance = 0.0;
    Serial.print("Filtered Sensed heading : ");
    Serial.println(State.FilteredSensedHeading);
    Serial.println("");
}

float Navigation::getFilteredHeading(float sensedHeading){
    float sum = 0.0;
	for (int i = 0; i < SIZE_HEADING_BUFFER-1; i++){
        sensedHeadingBuffer[i+1] = sensedHeadingBuffer[i];
	}
    sensedHeadingBuffer[0] = sensedHeading;

    for (int i=0; i < SIZE_HEADING_BUFFER; i++){
            sum += sensedHeadingBuffer[i];
    }
    return sum/SIZE_HEADING_BUFFER;
}

