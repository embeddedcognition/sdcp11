/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

#ifndef SIMULATORSERVER_H
#define SIMULATORSERVER_H

//includes
#include <string>
#include <uWS/uWS.h>
#include "json.hpp"
#include "requestprocessor.h"

//class declaration
class SimulatorServer
{
    public:
        //constructor
        SimulatorServer();
        //destructor
        virtual ~SimulatorServer();

        //function declaration
        //start the simulator server and inject the request processor dependency
        //the request processor is who the server forwards payloads (contained in messages received from the Udacity simulator)
        //to for processing and gets back information from it to send back to the Udacity simulator in response
        void start(RequestProcessor& request_processor);

    private:
        //function declaration
        //if a valid payload is found in the message, it is returned
        bool valid_message_payload_extracted(const string& message, json& payload);
};

#endif /* SIMULATORSERVER_H */
