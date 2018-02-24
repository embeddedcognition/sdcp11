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

    //request_message_payload[1] is the data JSON object

    // Main car's localization Data
    double car_x = request_message_payload[1]["x"];
    double car_y = request_message_payload[1]["y"];
    double car_s = request_message_payload[1]["s"];
    double car_d = request_message_payload[1]["d"];
    double car_yaw = request_message_payload[1]["yaw"];
    double car_speed = request_message_payload[1]["speed"];

    //cout << "car_d: " << car_d << endl;

    //cout << "map_waypoints length: " << map_waypoints.size() << endl;

    // Previous path data given to the Planner
    auto previous_path_x = request_message_payload[1]["previous_path_x"];
    auto previous_path_y = request_message_payload[1]["previous_path_y"];
    // Previous path's end s and d values
    double end_path_s = request_message_payload[1]["end_path_s"];
    double end_path_d = request_message_payload[1]["end_path_d"];

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    auto sensor_fusion = request_message_payload[1]["sensor_fusion"];





    //package and send back json

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
    response_message_payload["next_x"] = next_x_vals;
    response_message_payload["next_y"] = next_y_vals;

    return response_message_payload;
}
