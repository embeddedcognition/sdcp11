/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

#ifndef UTILITY_H
#define UTILITY_H

using namespace std;

//class declaration
class Waypoint
{
    public:
        double x;       //cartesian x coord of waypoint
        double y;       //cartesian y coord of waypoint
        double s;       //frenet s coord of waypoint
        double dx;      //x component to the vector normal/perpendicular to the waypoint
        double dy;      //y component to the vector normal/perpendicular to the waypoint
};

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
        vector<Waypoint> load_map_waypoints(string filename);

    private:
        //the current NIS for radar
        double NIS_radar_;
};

#endif /* UTILITY_H */
