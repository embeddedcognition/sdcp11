/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

//includes
#include "websocketserver.h"

//scoping
using std::string;
using std::size_t;
using std::cout;
using std::cerr;
using std::endl;
using std::function;
using json = nlohmann::json; //using type alias for convenience
using uWS::Hub;
using uWS::WebSocket;
using uWS::SERVER;
using uWS::OpCode;
using OpCode::TEXT;
using uWS::HttpRequest;

//class definition
//constructor
WebSocketServer::WebSocketServer() {}

//destructor
WebSocketServer::~WebSocketServer() {}

//function definition
//start the web socket server and inject the request processor dependency
//the telemetry processor is who the server forwards telmetry contained in client messages to for processing
//and gets back information from it to send back to the client in response
void WebSocketServer::start(RequestProcessor& request_processor)
{
    //local vars
    const int tcp_listen_port = 4567;                                                        //port the micro websocket server will listen on
    Hub hub;                                                                                 //micro websocket hub class
    function<void (WebSocket<SERVER>, char*, size_t, OpCode)> onMessage_lamda_func;          //lambda function for new message event
    function<void (WebSocket<SERVER>, HttpRequest)> onConnection_lamda_func;                 //lambda function for new connection event
    function<void (WebSocket<SERVER>, int, char*, size_t)> onDisconnection_lamda_func;       //lambda function for disconnect event

    /// define lambda functions ///
    //when a new message is received (capture reference to telemetry_processor and current object (to allow calling of valid_json_payload_extracted()) for use in the function)
    onMessage_lamda_func = [&request_processor, this] (WebSocket<SERVER> web_socket, char* request_message, size_t request_message_length, OpCode op_code)
    {
        //local vars
        json request_message_payload;       //object representation of the json structure for easy element extraction
        json response_message_payload;      //object representation of the json structure for easy element extraction
        string response_message;            //response message text string

        //the simulator and our server leverage a custom identifier ("42") at the beginning of messages to quickly identify them as websocket message events
        //the "4" signifies a websocket message, and the "2" signifies a websocket event
        //not sure why this is necessary (maybe to ensure other potential websocket client traffic is not confused as simulator events, regardless, the simulator requires it
        //the 42 is stripped off of the message and the remaining part is parsed for further usage

        //ensure the message contains data and that it has a 42 at the start, the remaining data after the first 2 bytes should be json (but we validate that)
        if ((request_message_length != 0) && (request_message_length > 2) && (request_message[0] == '4') && (request_message[1] == '2'))
        {
            //if a valid payload was found, send it to be processed
            if (valid_message_payload_extracted(request_message, request_message_payload))
            {
                //process request and return response
                response_message_payload = request_processor.compute_response_message_payload(request_message_payload);
                //format response message
                response_message = "42[\"control\"," + response_message_payload.dump() +"]";
                //send response message
                web_socket.send(response_message.data(), response_message.length(), TEXT);
            }
            else
            {
                //manual driving
                //format response message
                response_message = "42[\"manual\",{}]";
                //send response message
                web_socket.send(response_message.data(), response_message.length(), TEXT);
            }
        }
    };

    //when a new connection is received (capture no variables for use in the function)
    onConnection_lamda_func = [] (WebSocket<SERVER> web_socket, HttpRequest request)
    {
        cout << "Udacity simulator connected!!!" << endl;
    };

    //when a disconnect occurs (capture reference to hub for use in the function)
    onDisconnection_lamda_func = [&hub] (WebSocket<SERVER> web_socket, int code, char* request_message, size_t request_message_length)
    {
        cout << "Udacity simulator disconnected!!!" << endl;
        cout << "Shutting down..." << endl;
        //gracefully shutdown
        hub.getDefaultGroup<SERVER>().close();
        exit(0);
    };
    /// end define lambda functions ///

    //register lambda functions for particular events
    hub.onConnection(onConnection_lamda_func);        //when a new connection occurs, call this lambda function
    hub.onDisconnection(onDisconnection_lamda_func);  //when a disconnect occurs, call this lambda function
    hub.onMessage(onMessage_lamda_func);              //when a new message is received, call this lambda function

    //attempt to listen on the tcp port (the Udacity simulator will connect on this port)
    if (hub.listen(tcp_listen_port))
    {
        cout << "Listening to port: " << tcp_listen_port << endl;
        //start watching for and processing registered events (connect, message, disconnect, etc.) that come over the channel
        hub.run();
    }
    else
    {
        cerr << "Failed to listen to port: " << tcp_listen_port << endl;
    }
}

//function definition
//if a valid payload is found in the message, it is returned
bool WebSocketServer::valid_message_payload_extracted(const string& message, json& payload)
{
    //local vars
    size_t left_bracket_position;           //index position of "[", if found
    size_t right_curly_brace_postition;     //index position of "}", if found
    string text_payload;                    //raw payload as text string

    //message example:
    //42["telemetry",{"x":909.48,"y":1128.67,"yaw":0,"speed":0,"s":124.8336,"d":6.164833,"previous_path_x":[],"previous_path_y":[],"end_path_s":0,"end_path_d":0,
    //"sensor_fusion":[[0,1029.313,1148.76,19.30196,7.989753,244.4689,9.997465],[1,775.8,1425.2,0,0,6719.219,-280.1494],[2,775.8,1429,0,0,6716.599,-282.9019],
    //[3,775.8,1432.9,0,0,6713.911,-285.7268],[4,775.8,1436.3,0,0,6711.566,-288.1896],[5,775.8,1441.7,0,0,6661.772,-291.7797],[6,762.1,1421.6,0,0,6711.778,-268.0964],
    //[7,762.1,1425.2,0,0,6709.296,-270.7039],[8,762.1,1429,0,0,6663.543,-273.1828],[9,762.1,1432.9,0,0,6660.444,-275.5511],[10,762.1,1436.3,0,0,6657.743,-277.6157],
    //[11,762.1,1441.7,0,0,6653.453,-280.8947]]}]

    //locate any nulls in the message (if present, we ignore this message completely)
    //if we found a "null", return false
    if (message.find("null") != string::npos)
    {
        return false;
    }

    //if this is a telemetry event
    if (message.find("telemetry") != string::npos)
    {
        //validate json structure
        left_bracket_position = message.find_first_of("[");         //this should be the start of the structure (this also allows us to skip the 42 at the beginning)
        right_curly_brace_postition = message.find_first_of("}");   //since other right brackets could exist in the message, use the right curly brace to find the end

        //if we found signs of a json structure, return it
        if ((left_bracket_position != string::npos) && (right_curly_brace_postition != string::npos))
        {
            //capture the json payload
            //"right_curly_brace_postition - left_bracket_position + 2" will ensure the last right bracket (after the ending curly brace) is captured
            text_payload = message.substr(left_bracket_position, right_curly_brace_postition - left_bracket_position + 2);
            //convert text string to json object structure
            payload = json::parse(text_payload);
            //return success
            return true;
        }
    }

    //structure is incorrect, so return false
    return false;
}

