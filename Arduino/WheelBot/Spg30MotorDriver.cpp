#include "Spg30MotorDriver.h"

Spg30MotorDriver::Spg30MotorDriver(uint_least8_t motorPinA1,
                   uint_least8_t motorPinB1, uint_least8_t pwmPin, volatile long & encoderCount, int & motorSpeed) :
    ControlMode(IDLE),
    MotorSpeed(VEL_HIGH),
    _motorPinA1(motorPinA1),
    _motorPinB1(motorPinB1),
    _pwmPin(pwmPin),
    _encoderCount(encoderCount),
    _measuredSpeed(motorSpeed),
    _countInit(0),
    _tickNumber(0),
    _velocityCmd(0),
    _positionCmd(0),
    _driveForward(false),
    _driveBackward(false),
    _pwmCmd(0),
    _errorAccum(0),
    _lastMilliPrint(0),
    _Kp(5.0),
    _Ki(1.0),
    _motorIsRunning(false),
    _positionReached(true)
{
   _measuredSpeed = 0;
    /// - Initialize the Arduino pins for motor control and encoder readings.
    pinMode(_motorPinA1, OUTPUT);
    pinMode(_motorPinB1, OUTPUT);
    pinMode(_pwmPin, OUTPUT);
    analogWrite(_pwmPin, 0);
    digitalWrite(_motorPinA1, LOW);
    digitalWrite(_motorPinB1, HIGH);
}

void Spg30MotorDriver::run(){
    switch(ControlMode)
    {
        case POSITION:
            if(!_positionReached){
                _positionControl();
            }
            if(_motorIsRunning){
                if((abs(abs(_encoderCount)-abs(_countInit))) >= abs(_tickNumber)){
                    Serial.println("target reached");
                    ControlMode = IDLE;
                }
            }
        break;

        case VELOCITY:
            _pidControl();
        break;

        case IDLE:
            _positionReached = true;
            _positionCmd = 0;
            _encoderCount = 0;
            _motorBrake();
        break;

        default:
            /// - invalid state - go back to IDLE
            ControlMode = IDLE;
        break;
    }
}

void Spg30MotorDriver::_pidControl(){
    _updatePid();
    analogWrite(_pwmPin, _pwmCmd);
    //_printMotorInfo();
}

void Spg30MotorDriver::_updatePid(){
    float pidTerm = 0;
    int error=0;
    if (_velocityCmd < 0.0){
        digitalWrite(_motorPinA1, HIGH);
        digitalWrite(_motorPinB1, LOW);                  
	  }else {
        digitalWrite(_motorPinA1, LOW);
        digitalWrite(_motorPinB1, HIGH);    
	  }
	error = abs(_velocityCmd) - abs(_measuredSpeed); 
    error = constrain(error, -10, 10);
    _errorAccum += error;
    _errorAccum = constrain(_errorAccum, -150, 150);
    pidTerm = _pwmLookup[abs(_velocityCmd)] + (_Kp * error) + (_Ki * _errorAccum);
	_pwmCmd = constrain(int(pidTerm), 15, 255);
}

void Spg30MotorDriver::_positionControl(){
    if(_driveForward){
        _motorForward();
    }
    if(_driveBackward){
        _motorBackward();
    }
}

void Spg30MotorDriver::_motorForward(){
   _velocityCmd = MotorSpeed;
   _pidControl();
   _motorIsRunning = true;
}

void Spg30MotorDriver::_motorBackward(){
   _velocityCmd = -MotorSpeed;
   _pidControl();
   _motorIsRunning = true;
}

void Spg30MotorDriver::_motorBrake(){
   analogWrite(_pwmPin, STOP);
   digitalWrite(_motorPinA1, HIGH);
   digitalWrite(_motorPinB1, HIGH);
   _motorIsRunning = false;
}

void Spg30MotorDriver::_printMotorInfo(){  
  if((millis()-_lastMilliPrint) >= 500){                     
        _lastMilliPrint = millis();
        Serial.print("Error Accum:");    Serial.println(_errorAccum);  
        Serial.print("SP:");             Serial.println(_velocityCmd);  
        Serial.print("  RPM:");          Serial.println(_measuredSpeed);
        Serial.print("  PWM:");          Serial.println(_pwmCmd);
        Serial.println("");              
    }
}
