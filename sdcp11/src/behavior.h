/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <vector>

//class declaration
class Behavior
{
    public:
        //constructor
        Behavior();
        //destructor
        virtual ~Behavior();

        //function declaration
        //get the next best state to be in
        void get_next_state(const int);

    private:
        //the current NIS for radar
        double NIS_radar_;
};

#endif /* BEHAVIOR_H */
