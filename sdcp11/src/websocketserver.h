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
#include <uWS/uWS.h>
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
        //get the next best state to be in
        void get_next_state();
};

#endif /* WEBSOCKETSERVER_H */
