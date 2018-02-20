/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

#ifndef PATHPLANNER_H
#define PATHPLANNER_H

//includes
#include <string>
#include "json.hpp"
#include "guidancecontroller.h"

//scoping
using std::string;
using json = nlohmann::json; //using type alias for convenience

//class declaration
class PathPlanner : public GuidanceController
{
    public:
        //constructor
        PathPlanner();
        //destructor
        virtual ~PathPlanner();

        //function declaration
        //process telemetry and return guidance control (this is an override of a purely virtual function defined in the superclass "GuidanceController")
        json get_control_guidance_based_on_processed_telemetry(const json& telemetry);
};

#endif /* PATHPLANNER_H */
