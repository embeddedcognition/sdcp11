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
#include "json.hpp"
#include "requestprocessor.h"

//class declaration
class WebSocketServer
{
    public:
        //constructor
        WebSocketServer();
        //destructor
        virtual ~WebSocketServer();

        //function declaration
        //start the web socket server and inject the telemetry processor dependency
        //the request processor is who the server forwards payloads contained in client messages to for processing
        //and gets back information from it to send back to the client in response
        void start(RequestProcessor& request_processor);

    private:
        //function declaration
        //if a valid payload is found in the message, it is returned
        bool valid_message_payload_extracted(const string& message, json& payload);
};

#endif /* WEBSOCKETSERVER_H */
