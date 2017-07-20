#include "Navigation.h"

void Navigation::InitSensors() {
    for (int i = 0; i < SIZE_HEADING_BUFFER; i++){
        sensedHeadingBuffer[i] = 0.0;
    }    
}

void Navigation::Execute() {
    float sensedHeading = 0.0;
    if (State.ClosedLoopControl){
        sensedHeading = State.sensors.magnetometer.ReadHeading();
    }
    State.SensedHeading = getFilteredHeading(sensedHeading);
    avgError();
    Serial.print("Average heading error : ");
    Serial.println(State.AverageHeadingError);
    State.SensedDistance = 0.0;
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

float Navigation::avgError(){
    float sum = 0.0;
    for (int i = 0; i < SIZE_ERROR_BUFFER-1; i++){
        _errorBuffer[i+1] = _errorBuffer[i];
    }
    _errorBuffer[0] = State.HeadingError;

    for (int i=0; i < SIZE_ERROR_BUFFER; i++){
            sum += _errorBuffer[i];
    }
    State.AverageHeadingError = sum/SIZE_ERROR_BUFFER;
}


