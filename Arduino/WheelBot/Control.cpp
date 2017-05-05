#include "Control.h"

void Control::Execute() {
    //if (State.DoTestDrive){
        testDrive();
    //}
}

void Control::testDrive(){

    switch(TestDriveState)
    {
        case FWD_POS_TEST:
            if(firstPass){
                Serial.println("FWD POSITION TEST STARTING");
                State.effectors.rightMotor.FwdPositionCmd(720); 
                State.effectors.leftMotor.FwdPositionCmd(720); 
                firstPass=false;
            }
            /// - Keep running the controller until it's put back in to the IDLE
            ///   mode
            State.effectors.rightMotor.run();
            State.effectors.leftMotor.run();

            if (State.effectors.leftMotor.ReachedPosition() && 
                State.effectors.rightMotor.ReachedPosition()) {
                Serial.println("FWD POSITION TEST COMPLETE");
                firstPass = true;
                TestDriveState = BWD_POS_TEST;
            }

        break;

        case BWD_POS_TEST:
            if(firstPass){
                Serial.println("BWD POSITION TEST STARTING");
                State.effectors.rightMotor.BwdPositionCmd(720); 
                State.effectors.leftMotor.BwdPositionCmd(720); 
                firstPass=false;
            }
            /// - Keep running the controller until it's put back in to the IDLE
            ///   mode
            State.effectors.rightMotor.run();
            State.effectors.leftMotor.run();

            if (State.effectors.leftMotor.ReachedPosition() && 
                State.effectors.rightMotor.ReachedPosition()) {
                Serial.println("BWD POSITION TEST COMPLETE");
                Serial.println("VELOCITY TEST STARTING");
                firstPass = true;
                TestDriveState = VEL_TEST;
            }

        break;

        case VEL_TEST:
            /// - Used to ramp up the velocity command
            float lengthOfTestInSeconds = 5.0;
            float maxRpm = 40.0;
            numSecondsInTest += cycleTimeMillis/1000;
            /// - Set each motor contoller to velocity control mode.
            State.effectors.rightMotor.ControlMode = Spg30MotorDriver::VELOCITY; 
            State.effectors.leftMotor.ControlMode = Spg30MotorDriver::VELOCITY; 
            /// - Set the velocity command proportionally to the length of time
            ///   in this test velocity mode. R motor gets positive vel cmd,
            ///   L motor gets negative vel cmd
            int velocityCmd = (int) (numSecondsInTest/lengthOfTestInSeconds)*maxRpm;
            State.effectors.rightMotor.VelocityCmd(velocityCmd); 
            State.effectors.leftMotor.VelocityCmd(-1.0*velocityCmd); 
            /// - Run the motor controller until the end of the test
            if (numSecondsInTest < lengthOfTestInSeconds) {
                State.effectors.rightMotor.run();
                State.effectors.leftMotor.run();
            }else{
                State.effectors.rightMotor.ControlMode = Spg30MotorDriver::IDLE;
                State.effectors.leftMotor.ControlMode = Spg30MotorDriver::IDLE;
                Serial.println("VELOCITY TEST COMPLETE");
                TestDriveState = FWD_POS_TEST;
            }
        break;        
    }
}