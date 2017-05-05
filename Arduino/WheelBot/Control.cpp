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
                firstPass = true;
                TestDriveState = FWD_POS_TEST;
            }

        break;

    }
}