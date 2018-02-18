/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

//includes
#include <fstream>
#include <sstream>
#include "utility.h"

//namespaces
using std::ifstream;
using std::istringstream;

//class definition
//constructor
Utility::Utility() {}

//destructor
Utility::~Utility() {}

//function definition
//load the map waypoint data for the highway
void Utility::load_map_waypoints(vector<Waypoint>& map_waypoints, const string& file_location)
{
    //local vars
    Waypoint waypoint;              //structure to hold the current waypoint's information
    ifstream file_handle;           //handle to input file stream
    string cur_line;                //string to store current line of file
    istringstream parsed_line;      //stream that allows the current line to be parsed using white space (space, tab, etc.) as delimiter

    //open file
    file_handle.open(file_location);

    //if the file was successfully opened
    if (file_handle.is_open())
    {
        //loop through lines of file, using getline() to fetch a line into the "cur_line" buffer
        while (getline(file_handle, cur_line))
        {
            //clear the previous state of the string stream (as the str function will only reset it if it has already been cleared)
            parsed_line.clear();
            //put the current line into a stream that will allow it to be parsed via the white space (space, tab, etc.) between tokens
            parsed_line.str(cur_line);
            //extract tokens from left to right in the currnet line
            parsed_line >> waypoint.x;
            parsed_line >> waypoint.y;
            parsed_line >> waypoint.s;
            parsed_line >> waypoint.dx;
            parsed_line >> waypoint.dy;
            //push the current waypoint onto the vector
            map_waypoints.push_back(waypoint);
        }

        //close file
        file_handle.close();
    }

    //check to ensure file was read in correctly
    if (map_waypoints.size() == 0)
    {
        cout << "Error: Map file not read in correctly...exiting." << endl;
        exit(1);
    }
    else
    {
        //print out the x value of the first and last elements to validate the file was properly read
        cout << "Lines read in: " << map_waypoints.size() << endl;
        cout << "First line - x: " << (map_waypoints.front()).x << endl;
        cout << "Last line - x: " << (map_waypoints.back()).x << endl;
    }
}
