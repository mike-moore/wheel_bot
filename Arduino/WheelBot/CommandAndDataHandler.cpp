#include "CommandAndDataHandler.h"

CommandAndDataHandler::CommandAndDataHandler(CommandPacket& commands, TelemetryPacket& tlm, RobotState& state)
 :
 Commands(commands),
 Telemetry(tlm),
 State(state)
{}

CommandAndDataHandler::~CommandAndDataHandler() {}

void CommandAndDataHandler::ProcessCmds() {
    /// - Clear sent optional telemetry items from previous pass.
    ClearTelemetry();
    /// - Iterate over and process any rover commands that were sent
    for (uint_least8_t indx = 0; indx < Commands.RoverCmds_count; indx++){
        //Serial.println("Processing New Rover Command Received ... ");
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
    }
    if (rover_cmd.Id == STOP_TEST_DRIVE){
        State.DoTestDrive = false;
    }
    if(rover_cmd.Id == WP_GET_ACTIVE){
        Serial.println("WP GET ACTIVE");
        strncpy(Telemetry.ActiveWayPoint, State.ActiveWayPoint.Name, 15);
        Telemetry.has_ActiveWayPoint = true;
    }
}

void CommandAndDataHandler::ProcessWayPointCmd(WayPoint & way_point_cmd) {
    /// - Add the way point to the WayPointQueue
    if (State.WayPointQueue.count() < 15){
        Serial.println("adding waypoint to the queue.");
        State.WayPointQueue.push(way_point_cmd);
        /// - Pack the waypoint acknowledged command
        PackInt(WP_CMD_ACCEPT);
    }else{
        Serial.println("reject waypoint");
        /// - Pack the waypoint rejected command
        PackInt(WP_CMD_REJECT);       
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
    //if(SendResponseSignal){
    //    PackFloat(RESPONSE_SIGNAL, State.ResponseSignal);
    //    SendResponseSignal = false;
    //}
}

