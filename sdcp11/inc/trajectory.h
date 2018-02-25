/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

//includes
#include "spline.h"
#include "vehicletelemetry.h"

//class declaration
class Trajectory
{
    public:
        //constructor
        Trajectory();
        //destructor
        virtual ~Trajectory();

        //function declaration
        //based on the behavioral guidance, compute a vehicle trajectory
        void compute_vehicle_path(const VehicleTelemetry& vehicle_telemetry, const vector<Waypoint>& map_waypoints);
};

#endif /* TRAJECTORY_H */
