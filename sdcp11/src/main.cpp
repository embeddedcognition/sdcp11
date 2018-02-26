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
#include <stdlib.h>              //using for "EXIT_..." macros
#include "simulatorserver.h"
#include "pathplanner.h"

//function declarations
int main(const int, const char**);

//function definition
//main thread of execution
int main(const int argc, const char** argv)
{
    //local vars
    SimulatorServer simulator_server;   //communication agent between the Udacity simulator and the path planner module
    PathPlanner path_planner;           //path planning module (receives telemetry and returns a trajectory)

    // The max s value before wrapping around the track back to 0
    //double max_s = 6945.554;

    //start the simulator server, to listen for Udacity simulator connections,
    //once connected forward telemetry to the path planner and have it return back trajectories to the simulator
    simulator_server.start(path_planner);

    //exit program, clean return code
    return EXIT_SUCCESS;
}
