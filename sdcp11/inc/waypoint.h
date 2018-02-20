/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

#ifndef WAYPOINT_H
#define WAYPOINT_H

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

#endif /* WAYPOINT_H */
