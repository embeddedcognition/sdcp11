/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

#ifndef REQUESTPROCESSOR_H
#define REQUESTPROCESSOR_H

//includes
#include <string>
#include "json.hpp"

//namespaces
using std::string;
using json = nlohmann::json; //using type alias for convenience

//class declaration
class RequestProcessor
{
    public:
        //constructor
        RequestProcessor();
        //destructor
        virtual ~RequestProcessor();

        //function declaration
        //process supplied request and return response
        //make this an abstract class by making at least one purely virtual method (method = 0;)
        //making a normal method "purely virtual" forces the derived class to implement it
        virtual void compute_response_message_payload(const json& request_message_payload, json& response_message_payload) = 0;
};

#endif /* REQUESTPROCESSOR_H */
