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
            // - Monitor for manual drive command. 
            //   Initiates manual drive mode.
            if (State.ManualDriveMode){
                Mode = MANUAL_DRIVE_CTRL;
            }
        break;

        case ROTATIONAL_CTRL:
            State.effectors.rightMotor.run();
            State.effectors.leftMotor.run();
            if (State.effectors.leftMotor.ReachedPosition() && 
                State.effectors.rightMotor.ReachedPosition()) {
                State.HeadingReached = true;
                State.HeadingError = 0.0;
                Mode = IDLE;
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
                _testDriveState = DRIVE_FWD_OL;
            }

        break;

        case MANUAL_DRIVE_CTRL:
            State.effectors.leftMotor.VelocityCmd(State.CmdLeftMotorRpm);
            State.effectors.rightMotor.VelocityCmd(State.CmdRightMotorRpm);
            State.effectors.rightMotor.run();
            State.effectors.leftMotor.run();
            if (!State.ManualDriveMode){
                Mode = IDLE;
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
    }else{
        State.HeadingReached = true;
    }
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
        Serial.println(State.SensedHeading);
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

void Control::_testDrive(){
    switch(_testDriveState)
    {
        case DRIVE_FWD_OL:
            if(_firstPass){
                State.effectors.rightMotor.FwdPositionCmd(70020); 
                State.effectors.leftMotor.FwdPositionCmd(70020); 
                _firstPass=false;
            }
            State.effectors.rightMotor.run();
            State.effectors.leftMotor.run();
            if (State.effectors.leftMotor.ReachedPosition() && 
                State.effectors.rightMotor.ReachedPosition()) {
                _firstPass = true;
                _testDriveState = TURN_RIGHT_OL;
            }
        break;

        case TURN_RIGHT_OL:
            if(_firstPass){
                State.effectors.rightMotor.BwdPositionCmd(648); 
                State.effectors.leftMotor.FwdPositionCmd(648); 
                _firstPass=false;
            }
            State.effectors.rightMotor.run();
            State.effectors.leftMotor.run();

            if (State.effectors.leftMotor.ReachedPosition() && 
                State.effectors.rightMotor.ReachedPosition()) {
                _firstPass = true;
                _testDriveState = DRIVE_FWD_OL;
                _TurnRightCount++;
            }

            // - Swap over to closed loop after 2 turns
            if (_TurnRightCount >= 2){
                _TurnRightCount = 0;
                _firstPass = true;
                delay(10000);
                _testDriveState = DRIVE_FWD_CL;
            }

        break;

        case DRIVE_FWD_CL:
            if(_firstPass){
                State.ClosedLoopControl = true;
                State.effectors.rightMotor.ClosedLoopControl = true;
                State.effectors.leftMotor.ClosedLoopControl = true;
                State.effectors.rightMotor.FwdPositionCmd(720); 
                State.effectors.leftMotor.FwdPositionCmd(720); 
                _firstPass=false;
            }
            State.effectors.rightMotor.run();
            State.effectors.leftMotor.run();
            if (State.effectors.leftMotor.ReachedPosition() && 
                State.effectors.rightMotor.ReachedPosition()) {
                _firstPass = true;
                _testDriveState = TURN_RIGHT_CL;
            }
        break;

        case TURN_RIGHT_CL:
            if(_firstPass){
                State.effectors.rightMotor.BwdPositionCmd(648); 
                State.effectors.leftMotor.FwdPositionCmd(648); 
                _firstPass=false;
            }
            State.effectors.rightMotor.run();
            State.effectors.leftMotor.run();

            if (State.effectors.leftMotor.ReachedPosition() && 
                State.effectors.rightMotor.ReachedPosition()) {
                _firstPass = true;
                _testDriveState = DRIVE_FWD_CL;
                _TurnRightCount++;
            }

            // - Stop test drive after 2 closed loop turns
            if (_TurnRightCount >= 2){
                _TurnRightCount = 0;
                _testDriveState = DRIVE_FWD_OL;
                State.DoTestDrive = false;
                State.ClosedLoopControl = false;
                Mode = IDLE;
                State.effectors.rightMotor.ClosedLoopControl = false;
                State.effectors.leftMotor.ClosedLoopControl = false;
            }

        break;

    }
}
