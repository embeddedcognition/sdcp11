/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

#ifndef WEBSOCKETSERVER_H
#define WEBSOCKETSERVER_H

//includes
#include <string>
#include <uWS/uWS.h>
#include "guidancecontroller.h"
#include "json.hpp"

//class declaration
class WebSocketServer
{
    public:
        //constructor
        WebSocketServer();
        //destructor
        virtual ~WebSocketServer();

        //function declaration
        //start the web socket server and inject the message processor dependency
        //the guidance controller is who the server forwards telmetry contained in client messages to for processing
        //and gets back control commands from it to send back to the client in response
        void start(GuidanceController& guidance_controller);

    private:
        //function declaration
        //if a valid json structure is found in the message, it is returned
        bool valid_json_payload_extracted(const string& message, string& json_payload);
};

#endif /* WEBSOCKETSERVER_H */
