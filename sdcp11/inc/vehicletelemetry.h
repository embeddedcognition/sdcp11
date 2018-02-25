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
        double x;                                           //cartesian x coord of vehicle
        double y;                                           //cartesian y coord of vehicle
        double s;                                           //frenet s coord of vehicle
        double d;                                           //frenet d coord of vehicle
        double yaw;                                         //angle vehicle nose is pointing
        double speed;                                       //speed of the vehicle (in mph)
        //previous data supplied to the simulator
        vector<double> unconsumed_previous_path_x;          //list of x values from the previous path sent to the simulator (only contains values not yet consumed by the simulator)
        vector<double> unconsumed_previous_path_y;          //list of y values from the previous path sent to the simulator (only contains values not yet consumed by the simulator)
        double previous_path_ending_s;                      //frenet s coord of last point from previous path sent to the simulator
        double previous_path_ending_d;                      //frenet d coord of last point from previous path sent to the simulator
        //sensor fusion data for vehicles on same side of road as our vehicle
        //each element contains a vector of: [vehicle ID, vehicle x position in map coordinates, vehicle y position in map coordinates, vehicle x velocity in m/s,
        //                                    vehicle y velocity in m/s, vehicle s position in frenet coordinates, vehicle d position in frenet coordinates]
        vector<vector<double>> sensor_fusion;
};

#endif /* VEHICLETELEMETRY_H */
