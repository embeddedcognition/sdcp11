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
PathPlanner::PathPlanner() {}

//destructor
PathPlanner::~PathPlanner() {}

//function definition
//process telemetry and return guidance control
json PathPlanner::get_control_guidance_based_on_processed_telemetry(const json& telemetry)
{
    //local vars
    json return_val;


    // telemetry[1] is the data JSON object

                        // Main car's localization Data
                        double car_x = telemetry[1]["x"];
                        double car_y = telemetry[1]["y"];
                        double car_s = telemetry[1]["s"];
                        double car_d = telemetry[1]["d"];
                        double car_yaw = telemetry[1]["yaw"];
                        double car_speed = telemetry[1]["speed"];

                        //cout << "car_d: " << car_d << endl;

                        //cout << "map_waypoints length: " << map_waypoints.size() << endl;

                        // Previous path data given to the Planner
                        auto previous_path_x = telemetry[1]["previous_path_x"];
                        auto previous_path_y = telemetry[1]["previous_path_y"];
                        // Previous path's end s and d values
                        double end_path_s = telemetry[1]["end_path_s"];
                        double end_path_d = telemetry[1]["end_path_d"];

                        // Sensor Fusion Data, a list of all other cars on the same side of the road.
                        auto sensor_fusion = telemetry[1]["sensor_fusion"];









    //package and send back json

    //vector<double> next_x_vals;
                        //vector<double> next_y_vals;

                        // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                        //msgJson["next_x"] = next_x_vals;
                        //msgJson["next_y"] = next_y_vals;



    return return_val;
}
