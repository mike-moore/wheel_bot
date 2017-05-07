#include "Control.h"

void Control::Execute() {
    switch(Mode)
    {
        case IDLE:
            //_checkForDistanceControl();
            if (_headingError()){
                Mode = ROTATIONAL_CTRL;
            }
        break;

        case ROTATIONAL_CTRL:
            _performRotationControl();
            _printHeadingDebug();
            if (!_headingError()){
                Mode = IDLE;
                _velocityCmd = 0.0;
            }
            State.effectors.rightMotor.VelocityCmd(_velocityCmd); 
            State.effectors.rightMotor.VelocityCmd(-_velocityCmd); 
        break;

        case DRIVING_FWD:
            _positionCmd = 0.0;
            State.TargetReached = true;
            Mode = IDLE;
        break;

        case DRIVING_BWD:
            _positionCmd = 0.0;
            State.TargetReached = true;
            Mode = IDLE;
        break;

        default:
             /// - invalid state - go back to IDLE
             Mode = IDLE;
        break;
    }

    if (!_headingError()) { // && !_distanceError()){
        State.TargetReached = true;
        Mode = IDLE;
    }
}

bool Control::_headingError(){
    return (abs(State.HeadingError) > abs(State.HeadingErrorTol));
}

void Control::_checkForDistanceControl(){
   if (_distanceError()){
        if(State.DistanceError > 0.0){
            Mode = DRIVING_FWD;
        }else{
            Mode = DRIVING_BWD;
        }
    }
}

bool Control::_distanceError(){
    return (abs(State.DistanceError) > abs(State.DistanceErrorTol));
}

void Control::_printHeadingDebug(){
    Serial.print("Current heading : ");
    Serial.println(State.SensedHeading);
    Serial.print("Desired heading : ");
    Serial.println(State.ActiveWayPoint.Heading);
    Serial.print("Velocity Cmd : ");
    Serial.println(_velocityCmd);
}

void Control::_printDistanceDebug(){
    Serial.print("Sensed distance : ");
    Serial.println(State.SensedDistance);
    Serial.print("Distance error : ");
    Serial.println(State.DistanceError);
    Serial.print("Position Cmd : ");
    Serial.println(_positionCmd);
}

void Control::_performRotationControl(){
    float error = abs(State.HeadingError);
    _velocityCmd += _Kp*error + _Kd*error;
    _velocityCmd = constrain(_velocityCmd, -30, 30);
}

void Control::_testDrive(){

    float lengthOfTestInSeconds = 20.0;
    float maxRpm = 35.0;
    float velocityCmd = 0.0;
    switch(_testDriveState)
    {
        case FWD_POS_TEST:
            if(_firstPass){
                Serial.println("FWD POSITION TEST STARTING");
                State.effectors.rightMotor.FwdPositionCmd(720); 
                //State.effectors.leftMotor.FwdPositionCmd(720); 
                _firstPass=false;
            }
            /// - Keep running the controller until it's put back in to the IDLE
            ///   mode
            State.effectors.rightMotor.run();
            //State.effectors.leftMotor.run();

            if (State.effectors.leftMotor.ReachedPosition() && 
                State.effectors.rightMotor.ReachedPosition()) {
                Serial.println("FWD POSITION TEST COMPLETE");
                _firstPass = true;
                _testDriveState = BWD_POS_TEST;
            }

        break;

        case BWD_POS_TEST:
            if(_firstPass){
                Serial.println("BWD POSITION TEST STARTING");
                //State.effectors.rightMotor.BwdPositionCmd(720); 
                State.effectors.leftMotor.BwdPositionCmd(720); 
                _firstPass=false;
            }
            /// - Keep running the controller until it's put back in to the IDLE
            ///   mode
            State.effectors.rightMotor.run();
            State.effectors.leftMotor.run();

            if (State.effectors.leftMotor.ReachedPosition() && 
                State.effectors.rightMotor.ReachedPosition()) {
                Serial.println("BWD POSITION TEST COMPLETE");
                Serial.println("SPEED UP VELOCITY TEST STARTING");
                _firstPass = true;
                _testDriveState = SPEED_UP_VEL_TEST;
            }

        break;

        case SPEED_UP_VEL_TEST:
            /// - Used to ramp up the velocity command
            lengthOfTestInSeconds = 20.0;
            maxRpm = 30.0;
            _numSecondsInTest += (float) cycleTimeMillis/1000.0;
            /// - Set the velocity command proportionally to the length of time
            ///   in this test velocity mode. R motor gets positive vel cmd,
            ///   L motor gets negative vel cmd
            velocityCmd = _numSecondsInTest/lengthOfTestInSeconds*maxRpm;
            State.effectors.rightMotor.VelocityCmd(velocityCmd); 
            State.effectors.leftMotor.VelocityCmd(velocityCmd); 
            /// - Run the motor controller until the end of the test
            if (_numSecondsInTest < lengthOfTestInSeconds) {
                State.effectors.rightMotor.run();
                State.effectors.leftMotor.run();
            }else{
                State.effectors.rightMotor.ControlMode = Spg30MotorDriver::IDLE;
                State.effectors.leftMotor.ControlMode = Spg30MotorDriver::IDLE;
                Serial.println("SPEED UP VELOCITY TEST COMPLETE");
                _testDriveState = SLOW_DOWN_VEL_TEST;
                Serial.println("SLOW DOWN VELOCITY TEST STARTING");
                _numSecondsInTest = 0.0;
            }
        break; 
            
        case SLOW_DOWN_VEL_TEST:
            /// - Used to ramp up the velocity command
            lengthOfTestInSeconds = 20.0;
            maxRpm = 30.0;
            _numSecondsInTest += (float) cycleTimeMillis/1000.0;
            /// - Set the velocity command proportionally to the length of time
            ///   in this test velocity mode. R motor gets positive vel cmd,
            ///   L motor gets negative vel cmd
            velocityCmd = (1-_numSecondsInTest/lengthOfTestInSeconds)*maxRpm;
            State.effectors.rightMotor.VelocityCmd(velocityCmd); 
            State.effectors.leftMotor.VelocityCmd(velocityCmd); 
            /// - Run the motor controller until the end of the test
            if (_numSecondsInTest < lengthOfTestInSeconds) {
                State.effectors.rightMotor.run();
                State.effectors.leftMotor.run();
            }else{
                State.effectors.rightMotor.ControlMode = Spg30MotorDriver::IDLE;
                State.effectors.leftMotor.ControlMode = Spg30MotorDriver::IDLE;
                Serial.println("SLOW DOWN VELOCITY TEST COMPLETE");
                _testDriveState = FWD_POS_TEST;
                _numSecondsInTest = 0.0;
            }
        break;     
    }
}
