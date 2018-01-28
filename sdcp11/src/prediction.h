/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

#ifndef PREDICTION_H
#define PREDICTION_H

#include <vector>

class Prediction
{
    public:
        //constructor
        Prediction();
        //destructor
        virtual ~Prediction();

        //function declaration
        //compute the future states (locations) of the surrounding vehicles over a specific time horizon (in whole seconds)
        void compute_future_states_of_surrounding_vehicles(const int);

    private:
        //the current NIS for radar
        double NIS_radar_;
};

#endif /* PREDICTION_H */
