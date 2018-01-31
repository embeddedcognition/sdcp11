/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

#include "trajectory.h"

using namespace std;

//class definition
//constructor
Utility::Utility() {}

//destructor
Utility::~Utility() {}

//function definition
//load the map data for the highway
vector Utility::load_highway_map_data(string filename)
{
    //local vars
    // Waypoint map to read from
      string map_file_ = "../data/highway_map.csv";
      // The max s value before wrapping around the track back to 0
      double max_s = 6945.554;

      //instantiate input file stream class (which opens a stream to a file), feeding it the file path, and
      //ifstream in_map_(map_file_.c_str(), ifstream::in);
      ifstream in_map_(map_file_.c_str());

      //string to store current line of file
      string line;

      //loop through lines of file, using getline() to fetch a line into the "line" buffer
      while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
      }

      //check to ensure file was read in correctly
      std::cout << "map_waypoints_s size: " << map_waypoints_s.size() << std::endl;
      std::cout << "map_waypoints_x size: " << map_waypoints_x.size() << std::endl;
      std::cout << "map_waypoints_y size: " << map_waypoints_y.size() << std::endl;

      if ((map_waypoints_s.size() == 0) || (map_waypoints_x.size() == 0) || (map_waypoints_y.size() ==0))
      {
          std::cout << "Error: Map file not read in correctly...exiting." << std::endl;
          exit(1);
      }
}
