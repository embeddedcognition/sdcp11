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
};

#endif /* UTILITY_H */
