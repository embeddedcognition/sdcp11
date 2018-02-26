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
void PathPlanner::compute_response_message_payload(const json& request_message_payload, json& response_message_payload)
{
    //local vars
    VehicleTelemetry vehicle_telemetry;     //structure for easy access to vehicle telemetry
    vector<vector<double>> xy_path_points;  //contains two vectors, the first with the x values for the path and the second with the y values for the path

    //extract vehicle telemetry into more usable structure
    //request_message_payload[1] is the json object containing the telemetry data from the message payload
    //(request_message_payload[0] would contain the keyword "telemetry")
    extract_vehicle_telemetry(request_message_payload[1], vehicle_telemetry);

    //compute the vehicle's next path
    xy_path_points = trajectory_.compute_vehicle_path(vehicle_telemetry, map_waypoints_);

    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
    response_message_payload["next_x"] = xy_path_points[0];
    response_message_payload["next_y"] = xy_path_points[1];
}

//function definition
//extract vehicle telemetry from json payload structure into VehicleTelemetry structure for easier handling
void PathPlanner::extract_vehicle_telemetry(const json& telemetry_payload, VehicleTelemetry& vehicle_telemetry)
{
    //extract vehicle telemetry data
    //vehicle localization
    vehicle_telemetry.x = telemetry_payload["x"].get<double>();
    vehicle_telemetry.y = telemetry_payload["y"].get<double>();
    vehicle_telemetry.s = telemetry_payload["s"].get<double>();
    vehicle_telemetry.d = telemetry_payload["d"].get<double>();
    vehicle_telemetry.yaw = telemetry_payload["yaw"].get<double>();
    vehicle_telemetry.speed = telemetry_payload["speed"].get<double>();
    //previous path
    vehicle_telemetry.unconsumed_previous_path_x = telemetry_payload["previous_path_x"].get<vector<double>>();
    vehicle_telemetry.unconsumed_previous_path_y = telemetry_payload["previous_path_y"].get<vector<double>>();
    vehicle_telemetry.previous_path_ending_s = telemetry_payload["end_path_s"].get<double>();
    vehicle_telemetry.previous_path_ending_d = telemetry_payload["end_path_d"].get<double>();
    //sensor fusion
    vehicle_telemetry.sensor_fusion = telemetry_payload["sensor_fusion"].get<vector<vector<double>>>();
}
