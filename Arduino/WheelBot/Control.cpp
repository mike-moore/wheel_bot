#include "Control.h"

void Control::Execute() {
    if (State.DoTestDrive){
        testDrive();
    }
}

void Control::testDrive(){
    switch(TestDriveState)
    {
        case FWD_POS_TEST:
            State.effectors.rightMotor.ControlMode = Spg30MotorDriver::POSITION; 
            State.effectors.rightMotor.PositionCmd(720); 
            State.effectors.leftMotor.ControlMode = Spg30MotorDriver::POSITION; 
            State.effectors.leftMotor.PositionCmd(720); 

            if (State.effectors.rightMotor.ReachedPosition()) {
                State.effectors.rightMotor.ControlMode = Spg30MotorDriver::IDLE;
            }else{
                State.effectors.rightMotor.run();
            }

            if (State.effectors.leftMotor.ReachedPosition()) {
                State.effectors.leftMotor.ControlMode = Spg30MotorDriver::IDLE;
            }else{
                State.effectors.leftMotor.run();
            }

            if (State.effectors.leftMotor.ReachedPosition() && 
                State.effectors.rightMotor.ReachedPosition()) {
                Serial.println("FWD POSITION TEST COMPLETE");
                delay(5000);
                Serial.println("STARTING BWD POSITION TEST");
                TestDriveState = BWD_POS_TEST;
            }

        break;

        case BWD_POS_TEST:
            State.effectors.rightMotor.ControlMode = Spg30MotorDriver::POSITION; 
            State.effectors.rightMotor.PositionCmd(-720); 
            State.effectors.leftMotor.ControlMode = Spg30MotorDriver::POSITION; 
            State.effectors.leftMotor.PositionCmd(-720); 

            if (State.effectors.rightMotor.ReachedPosition()) {
                State.effectors.rightMotor.ControlMode = Spg30MotorDriver::IDLE;
            }else{
                State.effectors.rightMotor.run();
            }

            if (State.effectors.leftMotor.ReachedPosition()) {
                State.effectors.leftMotor.ControlMode = Spg30MotorDriver::IDLE;
            }else{
                State.effectors.leftMotor.run();
            }

            if (State.effectors.leftMotor.ReachedPosition() && 
                State.effectors.rightMotor.ReachedPosition()) {
                Serial.println("BWD POSITION TEST COMPLETE");
                delay(5000);
                Serial.println("STARTING FWD POSITION TEST");
                TestDriveState = FWD_POS_TEST;
            }

        break;

        case FWD_VEL_TEST:
            State.effectors.rightMotor.ControlMode = Spg30MotorDriver::VELOCITY; 
            State.effectors.rightMotor.VelocityCmd(35); 
            State.effectors.leftMotor.ControlMode = Spg30MotorDriver::VELOCITY; 
            State.effectors.leftMotor.VelocityCmd(35); 

            if (State.effectors.rightMotor.ReachedVelocity()) {
                State.effectors.rightMotor.ControlMode = Spg30MotorDriver::IDLE;
            }else{
                State.effectors.rightMotor.run();
            }

            if (State.effectors.leftMotor.ReachedVelocity()) {
                State.effectors.leftMotor.ControlMode = Spg30MotorDriver::IDLE;
            }else{
                State.effectors.leftMotor.run();
            }

            if (State.effectors.leftMotor.ReachedVelocity() && 
                State.effectors.rightMotor.ReachedVelocity()) {
                Serial.println("FWD VELOCITY TEST COMPLETE");
                delay(5000);
                Serial.println("STARTING ZERO VELOCITY TEST");
                TestDriveState = ZERO_VEL_TEST;
            }

        break;

        case ZERO_VEL_TEST:
            State.effectors.rightMotor.ControlMode = Spg30MotorDriver::VELOCITY; 
            State.effectors.rightMotor.VelocityCmd(0); 
            State.effectors.leftMotor.ControlMode = Spg30MotorDriver::VELOCITY; 
            State.effectors.leftMotor.VelocityCmd(0); 

            if (State.effectors.rightMotor.ReachedVelocity()) {
                State.effectors.rightMotor.ControlMode = Spg30MotorDriver::IDLE;
            }else{
                State.effectors.rightMotor.run();
            }

            if (State.effectors.leftMotor.ReachedVelocity()) {
                State.effectors.leftMotor.ControlMode = Spg30MotorDriver::IDLE;
            }else{
                State.effectors.leftMotor.run();
            }

            if (State.effectors.leftMotor.ReachedVelocity() && 
                State.effectors.rightMotor.ReachedVelocity()) {
                Serial.println("ZERO VELOCITY TEST COMPLETE");
                delay(5000);
                Serial.println("STARTING BWD VELOCITY TEST");
                TestDriveState = BWD_VEL_TEST;
            }

        break;

        case BWD_VEL_TEST:
            State.effectors.rightMotor.ControlMode = Spg30MotorDriver::VELOCITY; 
            State.effectors.rightMotor.VelocityCmd(-35); 
            State.effectors.leftMotor.ControlMode = Spg30MotorDriver::VELOCITY; 
            State.effectors.leftMotor.VelocityCmd(-35); 

            if (State.effectors.rightMotor.ReachedVelocity()) {
                State.effectors.rightMotor.ControlMode = Spg30MotorDriver::IDLE;
            }else{
                State.effectors.rightMotor.run();
            }

            if (State.effectors.leftMotor.ReachedVelocity()) {
                State.effectors.leftMotor.ControlMode = Spg30MotorDriver::IDLE;
            }else{
                State.effectors.leftMotor.run();
            }

            if (State.effectors.leftMotor.ReachedVelocity() && 
                State.effectors.rightMotor.ReachedVelocity()) {
                Serial.println("BWD VELOCITY TEST COMPLETE");
                delay(5000);
                Serial.println("STARTING FWD POSITION TEST");
                TestDriveState = FWD_POS_TEST;
            }

        break;

        default:
            /// - invalid state - go back to IDLE
            TestDriveState = FWD_POS_TEST;
        break;
    }
}