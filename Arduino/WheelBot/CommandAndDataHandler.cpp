#include "CommandAndDataHandler.h"

CommandAndDataHandler::CommandAndDataHandler(CommandPacket& commands, TelemetryPacket& tlm, RobotState& state)
 :
 Commands(commands),
 Telemetry(tlm),
 State(state),
 SendMotorRpms(false)
{}

CommandAndDataHandler::~CommandAndDataHandler() {}

void CommandAndDataHandler::ProcessCmds() {
    /// - Clear sent optional telemetry items from previous pass.
    ClearTelemetry();
    /// - Iterate over and process any rover commands that were sent
    for (uint_least8_t indx = 0; indx < Commands.RoverCmds_count; indx++){
        Serial.println("Processing New Rover Command Received ... ");
        ProcessRoverCmd(Commands.RoverCmds[indx]);
    }
    /// - Process the way point command if it was sent and valid
    ///   Invalid or no way-point sent defaults to 0 length string for its name
    if (strlen(Commands.WayPointCmd.Name)==0){
        return;
    }else{
        ProcessWayPointCmd(Commands.WayPointCmd);
    }
}

void CommandAndDataHandler::ClearTelemetry() {
    /// - Clear the count of Rover Status variables to be sent.
    ///   This is determined based on which RoverCmds are
    ///   received. It is incremented in LoadRoverStatus function.
    Telemetry.RoverStatus_count = 0;
    Telemetry.has_ActiveWayPoint = false;
}

void CommandAndDataHandler::LoadTelemetry() {
    /// - Load up all the required telemetry based on the robot's state.
    Telemetry.MeasuredHeading = State.SensedHeading;
    Telemetry.MeasuredDistance = State.SensedDistance;
    /// - Load the conditional telemetry. Data that has to be requested
    ///   in order to be sent.
    LoadRoverStatus();
}

void CommandAndDataHandler::ProcessRoverCmd(IdValuePairFloat & rover_cmd) {
    if (rover_cmd.Id == DO_TEST_DRIVE){
        Serial.println("STARTING TEST DRIVE");
        State.DoTestDrive = true;
        PackInt(CMD_ACCEPT);
    }
    if (rover_cmd.Id == STOP_TEST_DRIVE){
        State.DoTestDrive = false;
        PackInt(CMD_ACCEPT);
    }
    if(rover_cmd.Id == WP_GET_ACTIVE){
        Serial.println("WP GET ACTIVE");
        PackInt(CMD_ACCEPT);
        strncpy(Telemetry.ActiveWayPoint, State.ActiveWayPoint.Name, 15);
        Telemetry.has_ActiveWayPoint = true;
    }
    if(rover_cmd.Id == GET_MOTOR_DATA){
        Serial.println("GET MOTOR DATA CMD");
        PackInt(CMD_ACCEPT);
        SendMotorRpms = true;
    }
    if(rover_cmd.Id == MANUAL_DRIVE){
        Serial.println("ENABLE MANUAL DRIVE MODE");
        State.ManualDriveMode = true;
        PackInt(CMD_ACCEPT);
    }
    if(rover_cmd.Id == MANUAL_DRIVE_STOP){
        Serial.println("DISABLE MANUAL DRIVE MODE");
        State.ManualDriveMode = false;
        PackInt(CMD_ACCEPT);
    }
    if(rover_cmd.Id == CMD_L_MOTOR_RPM){
        Serial.println("CMD L MOTOR RPM");
        if (State.ManualDriveMode == true){
            PackInt(CMD_ACCEPT);
            State.CmdLeftMotorRpm = rover_cmd.Value;
        }else{
            PackInt(CMD_REJECT);
        }
    }
    if(rover_cmd.Id == CMD_R_MOTOR_RPM){
        Serial.println("CMD R MOTOR RPM");
        if (State.ManualDriveMode == true){
            PackInt(CMD_ACCEPT);
            State.CmdRightMotorRpm = rover_cmd.Value;
        }else{
            PackInt(CMD_REJECT);
        }
    }
}

void CommandAndDataHandler::ProcessWayPointCmd(WayPoint & way_point_cmd) {
    /// - Add the way point to the WayPointQueue
    if (State.WayPointQueue.count() < 15){
        Serial.println("adding waypoint to the queue.");
        State.WayPointQueue.push(way_point_cmd);
        PackInt(CMD_ACCEPT);
    }else{
        Serial.println("reject waypoint");
        PackInt(CMD_REJECT);       
    }
}

void CommandAndDataHandler::PackInt(uint32_t id) {
    Telemetry.RoverStatus[Telemetry.RoverStatus_count].Id = id;
    Telemetry.RoverStatus_count++;
}

void CommandAndDataHandler::PackFloat(uint32_t id, float value) {
    Telemetry.RoverStatus[Telemetry.RoverStatus_count].Id = id;
    Telemetry.RoverStatus[Telemetry.RoverStatus_count].Value = value;
    Telemetry.RoverStatus[Telemetry.RoverStatus_count].has_Value = true;
    Telemetry.RoverStatus_count++;
}

void CommandAndDataHandler::LoadRoverStatus() {
    if(SendMotorRpms){
        PackFloat(L_MOTOR_COUNT, State.LeftMotorCount);
        PackFloat(R_MOTOR_COUNT, State.RightMotorCount);
        PackFloat(L_MOTOR_RPM, State.LeftMotorRpm);
        PackFloat(R_MOTOR_RPM, State.RightMotorRpm);
        SendMotorRpms = false;
    }
}

