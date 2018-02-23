/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"

//includes
#include "simulatorserver.h"
#include "pathplanner.h"

//function declarations
int main(const int, const char**);

//function definition
//main thread of execution
int main(const int argc, const char** argv)
{
    //local vars
    SimulatorServer simulator_server;
    PathPlanner path_planner;

    // The max s value before wrapping around the track back to 0
    //double max_s = 6945.554;

    //start the simulator server, to listen for Udacity simulator connections,
    //once connected forward telemetry to the path planner and have it return back trajectories to the simulator
    simulator_server.start(path_planner);

    return 0;
}
