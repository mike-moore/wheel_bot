#include "Navigation.h"

void Navigation::InitSensors() {
    State.sensors.magnetometer.Init();
    for (int i = 0; i < SIZE_HEADING_BUFFER; i++){
        sensedHeadingBuffer[i] = 0.0;
    }    
}

void Navigation::Execute() {
    float sensedHeading = 0.0;
    sensedHeading = State.sensors.magnetometer.ReadHeading();
    State.SensedHeading = getFilteredHeading(sensedHeading);
    avgError();
    State.SensedDistance = 0.0;
    State.LeftMotorCount = State.effectors.leftMotor.getCount();
    State.RightMotorCount = State.effectors.rightMotor.getCount();
    State.LeftMotorRpm = State.effectors.leftMotor.getSpeed();
    State.RightMotorRpm = State.effectors.rightMotor.getSpeed();
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

void Navigation::avgError(){
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


