/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

//#include <math.h>
//#include <uWS/uWS.h>
//#include <chrono>
//#include <thread>
//#include <vector>
//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
//#include "json.hpp"
//#include "spline.h"

//includes
#include "websocketserver.h"
#include "pathplanner.h"
#include "utility.h"

//function declarations
int main(const int, const char**);

//function definition
//main thread of execution
int main(const int argc, const char** argv)
{
    //local vars

    //Utility utility;                                                                                        //utility class with helper functions
    //vector<Waypoint> map_waypoints;                                                                         //map waypoints associated with highway in the simulator
    //const string file_location = "../data/highway_map.csv";                                                 //path to map waypoint data
    WebSocketServer web_socket_server;
    PathPlanner path_planner;

    // The max s value before wrapping around the track back to 0
    //double max_s = 6945.554;

    //load map waypoints for the simulator highway we're driving on
    //utility.load_map_waypoints(map_waypoints, file_location);

    //start the web socket server, to listen for simulator connections, once connected send telemetry to the path planner and have it return back trajectories to the simulator
    web_socket_server.start(path_planner);

    return 0;
}
