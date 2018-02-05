/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

//#include <fstream>
//#include <math.h>
#include <uWS/uWS.h>
//#include <chrono>
//#include <iostream>
//#include <thread>
//#include <vector>
//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
//#include "spline.h"

//includes
#include "utility.h"

//namespaces
using namespace std;
using json = nlohmann::json; //shortened for convenience

//function declarations
int main(const int, const char**);
string get_data(string s);

//function definition
//main thread of execution
int main(const int argc, const char** argv)
{
    //local vars
    uWS::Hub hub;                                                                                           //micro websocket hub class
    function<void (uWS::WebSocket<uWS::SERVER>, char*, size_t, uWS::OpCode)> onMessage_lamda_func;          //lambda function for new message event
    function<void (uWS::WebSocket<uWS::SERVER>, uWS::HttpRequest)> onConnection_lamda_func;                 //lambda function for new connection event
    function<void (uWS::WebSocket<uWS::SERVER>, int, char*, size_t)> onDisconnection_lamda_func;            //lambda function for disconnect event
    function<void (uWS::HttpResponse*, uWS::HttpRequest, char*, size_t, size_t)> onHttpRequest_lamda_func;  //lambda function for http request event
    //Utility utility;                                                                                        //utility class with helper functions
    vector<Waypoint> map_waypoints;                                                                         //map waypoints associated with highway in the simulator
    const int tcp_listen_port = 4567;                                                                       //port the micro websocket server will listen on
    const string file_location = "../data/highway_map.csv";                                                 //path to map waypoint data

    //load map waypoints for the simulator highway we're driving on
    //utility.load_map_waypoints(map_waypoints, file_location);

    /// define lambda functions ///
    //when a new message is received (capture reference to map_waypoints vector for use in the function)
    onMessage_lamda_func = [&map_waypoints] (uWS::WebSocket<uWS::SERVER> ws, char* message, size_t length, uWS::OpCode opCode)
    {
        //"42" at the start of the message means there's a websocket message event.
        //the 4 signifies a websocket message
        //the 2 signifies a websocket event
        if ((length != 0) && (length > 2) && (message[0] == '4') && (message[1] == '2'))
        {
            //check if the message has data
            auto s = get_data(message);

            if (s != "")
            {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry")
                {
                  // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    cout << "car_d: " << car_d << endl;

                    cout << "map_waypoints length: " << map_waypoints.size() << endl;

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
            else
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    };

    //when a new connection is received (capture no variables for use in the function)
    onConnection_lamda_func = [] (uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        cout << "Udacity simulator connected!!!" << endl;
    };

    //when a disconnect occurs (capture reference to hub for use in the function)
    onDisconnection_lamda_func = [&hub] (uWS::WebSocket<uWS::SERVER> ws, int code, char* message, size_t length)
    {
        cout << "Udacity simulator disconnected!!!" << endl;
        cout << "Shutting down..." << endl;
        //gracefully shutdown the server
        hub.getDefaultGroup<uWS::SERVER>().close();
    };

    //when a new http request is received (capture no variables for use in the function)
    onHttpRequest_lamda_func = [] (uWS::HttpResponse* res, uWS::HttpRequest req, char* data, size_t length, size_t remainingBytes)
    {
        cout << "New HTTP request received!!!" << endl;
        const string return_message = "<h1>Hello World!</h1>";
        res->end(return_message.data(), return_message.length());
    };
    /// end define lambda functions ///

    //register lambda functions for particular events
    hub.onConnection(onConnection_lamda_func);        //when a new connection occurs, call this lambda function
    hub.onDisconnection(onDisconnection_lamda_func);  //when a disconnect occurs, call this lambda function
    hub.onMessage(onMessage_lamda_func);              //when a new message is received, call this lambda function
    //hub.onHttpRequest(onHttpRequest_lamda_func);      //when a new http request is received, call this lambda function

    //attempt to listen on the tcp port (the Udacity simulator will connect to this port)
    if (hub.listen(tcp_listen_port))
    {
        cout << "Listening to port: " << tcp_listen_port << endl;
        //start thread to watch for and process registered events (connect, message, disconnect, etc.) that come over the channel
        hub.run();
    }
    else
    {
        cerr << "Failed to listen to port: " << tcp_listen_port << endl;
        return -1;
    }

    return 0;
}

//function definition
//checks if the socketio event has json data, if so the json object in string format will be returned, else the empty string "" will be returned
string get_data(string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos)
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}
