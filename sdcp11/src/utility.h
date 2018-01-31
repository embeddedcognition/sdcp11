/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

#ifndef UTILITY_H
#define UTILITY_H

//class declaration
class Utility
{
    public:
        //constructor
        Utility();
        //destructor
        virtual ~Utility();

        //function declaration
        //get the next best state to be in
        void get_next_state(const int);

    private:
        //the current NIS for radar
        double NIS_radar_;
};

#endif /* UTILITY_H */
