/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

#ifndef VEHICLETELEMETRY_H
#define VEHICLETELEMETRY_H

//class declaration
class VehicleTelemetry
{
    public:
        //vehicle localization
        double x;       //cartesian x coord of vehicle
        double y;       //cartesian y coord of vehicle
        double s;       //frenet s coord of vehicle
        double d;       //frenet d coord of vehicle
        double yaw;     //angle vehicle nose is pointing
};

#endif /* VEHICLETELEMETRY_H */
