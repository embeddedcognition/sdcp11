/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

#ifndef GUIDANCECONTROLLER_H
#define GUIDANCECONTROLLER_H

//includes
#include <string>
#include "json.hpp"

//namespaces
using std::string;
using json = nlohmann::json; //using type alias for convenience

//class declaration
class GuidanceController
{
    public:
        //constructor
        GuidanceController();
        //destructor
        virtual ~GuidanceController();

        //function declaration
        //process telemetry and return guidance control
        //make this an abstract class by making at least one purely virtual method (method = 0;)
        //making a normal method "purely virtual" forces the derived class to implement it
        virtual json get_control_guidance_based_on_processed_telemetry(const json& telemetry) = 0;
};

#endif /* GUIDANCECONTROLLER_H */
