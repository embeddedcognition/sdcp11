/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

#ifndef UTILITY_H
#define UTILITY_H

//includes
#include <iostream>
#include <string>
#include <vector>
#include "waypoint.h"

//scoping
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using std::string;

//class declaration
class Utility
{
    public:
        //constructor
        Utility();
        //destructor
        virtual ~Utility();

        //function declaration
        //load the map waypoint data for the highway
        void load_map_waypoints(vector<Waypoint>& map_waypoints, const string& file_location);

        //function declaration
        //convert degrees to radians
        double convert_degrees_to_radians(const double degrees);

        //function declaration
        //convert frenet to cartesian
        vector<double> convert_frenet_to_cartesian(double s, double d, const vector<Waypoint>& map_waypoints);

        //function declaration
        //compute distance between two points
        double compute_distance_between_points(double x1, double y1, double x2, double y2);
};

#endif /* UTILITY_H */
