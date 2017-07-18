#include "Control.h"

void Control::Execute() {
    switch(Mode)
    {
        case IDLE:
            State.HeadingReached = false;
            State.DistanceReached = false;
            // - Keep motors braked in idle mode.
            State.effectors.rightMotor.MotorBrake();
            State.effectors.leftMotor.MotorBrake();
            // - Check for an active heading error
            _checkForHeadingControl();
            // - Check for an active distance error only once heading
            //   error has been closed out
            if(!_headingError()){
                _checkForDistanceControl();
            }
            // - Monitor for test drive command. 
            //   Initiates open loop sequence.
            if (State.DoTestDrive){
                Mode = TEST_DRIVE;
            }
        break;

        case ROTATIONAL_CTRL:
            State.effectors.rightMotor.run();
            State.effectors.leftMotor.run();
            if(filteredError <= 5.0){
                if (State.effectors.leftMotor.ReachedPosition() && 
                    State.effectors.rightMotor.ReachedPosition()) {
                    State.HeadingReached = true;
                    State.HeadingError = 0.0;
                    Mode = IDLE;
                }
            }
        break;

        case TRANSLATIONAL_CTRL:
            State.effectors.rightMotor.run();
            State.effectors.leftMotor.run();
            if (State.effectors.leftMotor.ReachedPosition() && 
                State.effectors.rightMotor.ReachedPosition()) {
                State.DistanceReached = true;
                State.DistanceError = 0.0;
                Mode = IDLE;
            }

        break;
        
        case TEST_DRIVE:
            _testDrive();
            if (!State.DoTestDrive){
                Mode = IDLE;
                _testDriveState = FWD_POS_TEST;
            }

        break;

        default:
             /// - invalid state - go back to IDLE
             Mode = IDLE;
        break;
    }

}

bool Control::_headingError(){
    return (abs(State.HeadingError) > abs(State.HeadingErrorTol));
}

void Control::_checkForHeadingControl(){
    if (_headingError()){
        uint16_t pos_cmd = 0;
        if(State.HeadingError > 0.0){
            pos_cmd = abs(State.HeadingError)*MotorRotDegPerDegHeading; 
            State.effectors.rightMotor.BwdPositionCmd(pos_cmd); 
            State.effectors.leftMotor.FwdPositionCmd(pos_cmd); 
            Mode = ROTATIONAL_CTRL;
        }else{
            pos_cmd = abs(State.HeadingError)*MotorRotDegPerDegHeading; 
            State.effectors.rightMotor.FwdPositionCmd(pos_cmd); 
            State.effectors.leftMotor.BwdPositionCmd(pos_cmd); 
            Mode = ROTATIONAL_CTRL;
        }
        if(State.HeadingError >= 45.0){
            velocityRight
            State.effectors.rightMotor.VelocityCmd(velocityRight); 
            State.effectors.leftMotor.VelocityCmd(velocityLeft);
            Mode = ROTATIONAL_CTRL;
        }
    else{
        State.HeadingReached = true;
    }
    }
}

float Control::_pdHeadingControl(){
    float filteredError = 0.0;

    _Kp_Left = 0.45;
    _Kp_Right = 0.506;
    _Kd_Left = 0.05; 
    _Kd_Right = 0.05;
            
    float error = State.HeadingError;
    errorRate = error - prev_error;
    filteredError = getFilteredError(abs(error));
            
    velocityRight = -_Kp_Right*error + _Kd_Right*errorRate;
    velocityLeft = _Kp_Left*error - _Kd_Left*errorRate;
    velocityRight = constrain(velocityRight, -30, 30);
    velocityLeft = constrain(velocityLeft, -30, 30);

    prev_error = error;
    return;
}

float Control::getFilteredError(float error){
    float sum = 0.0;
    for (int i = 0; i < SIZE_ERROR_BUFFER-1; i++){
        errorBuffer[i+1] = errorBuffer[i];
    }
    errorBuffer[0] = error;

    for (int i = 0; i < SIZE_ERROR_BUFFER; i++){
            sum += errorBuffer[i];
    }
    return sum/SIZE_ERROR_BUFFER;
}
   
void Control::_checkForDistanceControl(){
   if (_distanceError()){
        uint16_t pos_cmd = 0;
        if(State.DistanceError > 0.0){
            pos_cmd = abs(State.DistanceError)*MotorRotDegPerFt; 
            State.effectors.rightMotor.FwdPositionCmd(pos_cmd); 
            State.effectors.leftMotor.FwdPositionCmd(pos_cmd); 
            Mode = TRANSLATIONAL_CTRL;
        }else{
            pos_cmd = abs(State.DistanceError)*MotorRotDegPerFt; 
            State.effectors.rightMotor.BwdPositionCmd(pos_cmd); 
            State.effectors.leftMotor.BwdPositionCmd(pos_cmd); 
            Mode = TRANSLATIONAL_CTRL;
        }
    }else{
        State.DistanceReached = true;
    }
}

bool Control::_distanceError(){
    return (abs(State.DistanceError) > abs(State.DistanceErrorTol));
}

void Control::_printHeadingDebug(){
  if((millis()-_lastMilliPrint) >= 2000){                     
        _lastMilliPrint = millis();
        Serial.print("Current heading : ");
        Serial.println(State.FilteredSensedHeading);
        Serial.print("Desired heading : ");
        Serial.println(State.ActiveWayPoint.Heading);
        Serial.print("Velocity Cmd : ");
        Serial.println(_velocityCmd);            
    }
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
                State.effectors.leftMotor.FwdPositionCmd(720); 
                _firstPass=false;
            }
            /// - Keep running the controller until it's put back in to the IDLE
            ///   mode
            State.effectors.rightMotor.run();
            State.effectors.leftMotor.run();

            if (State.effectors.leftMotor.ReachedPosition() && 
                State.effectors.rightMotor.ReachedPosition()) {
                Serial.println("FWD POSITION TEST COMPLETE");
                _firstPass = true;
                _testDriveState = TURN_RIGHT_TEST;
            }

        break;

        case TURN_RIGHT_TEST:
            if(_firstPass){
                Serial.println("RIGHT TURN POSITION TEST STARTING");
                State.effectors.rightMotor.BwdPositionCmd(305); 
                State.effectors.leftMotor.FwdPositionCmd(305); 
                _firstPass=false;
            }
            /// - Keep running the controller until it's put back in to the IDLE
            ///   mode
            State.effectors.rightMotor.run();
            State.effectors.leftMotor.run();

            if (State.effectors.leftMotor.ReachedPosition() && 
                State.effectors.rightMotor.ReachedPosition()) {
                Serial.println("RIGHT TURN POSITION TEST COMPLETE");
                Serial.println("HEADING CONTROL TEST STARTING");
                //Serial.println("SPEED UP VELOCITY TEST STARTING");
                _firstPass = true;
                //_testDriveState = SPEED_UP_VEL_TEST;
                _testDriveState = HEADING_CONTROL_TEST;
            }

        break;

        case SPEED_UP_VEL_TEST:
            /// - Used to ramp up the velocity command
            lengthOfTestInSeconds = 20.0;
            maxRpm = 10.0;
            _numSecondsInTest += (float) cycleTimeControl/1000.0;
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
            maxRpm = 10.0;
            _numSecondsInTest += (float) cycleTimeControl/1000.0;
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
                _testDriveState = SPEED_UP_VEL_TEST;
                _numSecondsInTest = 0.0;
            }
        break;

        case HEADING_CONTROL_TEST:
            float desiredHeading = 180.0;
            float filteredError = 0.0;
            State.HeadingError = desiredHeading - State.FilteredSensedHeading;
            Serial.print("Sensed heading : ");
            Serial.println(State.FilteredSensedHeading);
            Serial.println("");

            Serial.print("Heading Error : ");
            Serial.println(State.HeadingError);
            Serial.println("");

            _Kp_Left = 0.45;
            _Kp_Right = 0.506;
            _Kd_Left = 0.05; 
            _Kd_Right = 0.05;
            
            float error = State.HeadingError;
            errorRate = error - prev_error;
            filteredError = getFilteredError(abs(error));
            
            velocityRight = -_Kp_Right*error + _Kd_Right*errorRate;
            velocityLeft = _Kp_Left*error - _Kd_Left*errorRate;
            velocityRight = constrain(velocityRight, -30, 30);
            velocityLeft = constrain(velocityLeft, -30, 30);
            Serial.print("Velocity Right : ");
            Serial.println(velocityRight);
            Serial.print("Velocity Left : ");
            Serial.println(velocityLeft);
            Serial.println("");

            State.effectors.rightMotor.VelocityCmd(velocityRight); 
            State.effectors.leftMotor.VelocityCmd(velocityLeft);
   
            State.effectors.rightMotor.run();
            State.effectors.leftMotor.run();

            //if(filteredError <= 5.0){
            if (State.effectors.leftMotor.ReachedPosition() && 
                State.effectors.rightMotor.ReachedPosition())
                State.effectors.rightMotor.ControlMode = Spg30MotorDriver::IDLE;
                State.effectors.leftMotor.ControlMode = Spg30MotorDriver::IDLE;
            //}
            
            //_testDriveState = TURN_RIGHT_TEST;
            prev_error = error;
        break;    
    }
}
