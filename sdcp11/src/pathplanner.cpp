/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

#include "pathplanner.h"

//class definition
//constructor
PathPlanner::PathPlanner()
{
    //load map waypoints for the simulator highway we're driving on
    utility_.load_map_waypoints(map_waypoints_, file_location_);
}

//destructor
PathPlanner::~PathPlanner() {}

//function definition
//process telemetry and return guidance control
json PathPlanner::compute_response_message_payload(const json& request_message_payload)
{
    //local vars
    json response_message_payload;
    VehicleTelemetry vehicle_telemetry;

    //extract vehicle telemetry into more usable structure
    //request_message_payload[1] is the json object containing the telemetry data from the message payload
    //(request_message_payload[0] would contain the keyword "telemetry")
    extract_vehicle_telemetry(request_message_payload[1], vehicle_telemetry);

    //compute the vehicle's next path
    trajectory_.compute_vehicle_path(vehicle_telemetry, map_waypoints_);

    //package and send back json

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
    response_message_payload["next_x"] = next_x_vals;
    response_message_payload["next_y"] = next_y_vals;

    return response_message_payload;
}

//function definition
//extract vehicle telemetry from json payload structure into VehicleTelemetry structure for easier handling
void PathPlanner::extract_vehicle_telemetry(const json& telemetry_payload, VehicleTelemetry& vehicle_telemetry)
{
    //extract vehicle telemetry data
    //vehicle localization
    vehicle_telemetry.x = telemetry_payload["x"];
    vehicle_telemetry.y = telemetry_payload["y"];
    vehicle_telemetry.s = telemetry_payload["s"];
    vehicle_telemetry.d = telemetry_payload["d"];
    vehicle_telemetry.yaw = telemetry_payload["yaw"];
    vehicle_telemetry.speed = telemetry_payload["speed"];
    //previous path
    vehicle_telemetry.unconsumed_previous_path_x = telemetry_payload["previous_path_x"];
    vehicle_telemetry.unconsumed_previous_path_y = telemetry_payload["previous_path_y"];
    vehicle_telemetry.previous_path_ending_s = telemetry_payload["end_path_s"];
    vehicle_telemetry.previous_path_ending_d = telemetry_payload["end_path_d"];
    //sensor fusion
    vehicle_telemetry.sensor_fusion = telemetry_payload["sensor_fusion"];
}
