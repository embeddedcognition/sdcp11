/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>

//class declaration
class Trajectory
{
    public:
        //constructor
        Trajectory();
        //destructor
        virtual ~Trajectory();

        //function declaration
        //get the next best state to be in
        void get_next_state(const int);

    private:
        //the current NIS for radar
        double NIS_radar_;
};

#endif /* TRAJECTORY_H */
