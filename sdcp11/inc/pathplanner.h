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
#include <vector>
#include <string>
#include "json.hpp"
#include "requestprocessor.h"

//scoping
using std::vector;
using std::string;
using json = nlohmann::json; //using type alias for convenience

//class declaration
class PathPlanner : public RequestProcessor
{
    public:
        //constructor
        PathPlanner();
        //destructor
        virtual ~PathPlanner();

        //function declaration
        //process vehicle telemetry and return guidance/trajectory (this is an override of a purely virtual function defined in the superclass "TelemetryProcessor")
        json compute_response_message_payload(const json& request_message_payload);
};

#endif /* PATHPLANNER_H */
