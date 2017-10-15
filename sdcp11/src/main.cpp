#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	std::cout << "car_d: " << car_d << std::endl;

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	//indicates the lane we want to be in (left = 0, middle = 1, right = 2)
          	int target_lane = 1;
          	//define a reference velocity that acts as the speed we want to hover around but not exceed
          	double reference_velocity = 49.5;
          	//capture the size of the previous set of path points that were not reached in the last iteration (the remaining points at the end of the previous plan)
          	int previous_path_size = previous_path_x.size();

          	//create a list of widely spaced (x, y) anchor points, evenly spaced apart at 30 meters
          	//we'll interpolate these points using the spline library to fill in the gaps at the spacing
          	//we need to maintain our reference velocity (remember the spacing determines acceleration)
          	vector<double> anchor_points_x;
          	vector<double> anchor_points_y;

          	//establish reference x, y, yaw states
          	//on the first iteration we'll not have a previous set of path points to extend upon so we need to start with the vehicle's current location
          	double reference_x = car_x;
          	double reference_y = car_y;
          	double reference_yaw = deg2rad(car_yaw);

          	//if the previous path size contains less then 2 points, we'll need to use the vehicle's position as a starting reference
          	//we'll create a second point (previous to the vehicle's current position) based on the vehicle's current yaw
          	if (previous_path_size < 2)
          	{
          	    //compute two anchor points (the vehicle's current position and a point before that) to use since we don't have a previous path to look at
          	    //we use the vehicle's yaw angle to ensure the previous point is on the same line as the vehicle's angle
          	    double previous_car_x = car_x - cos(car_yaw);
          	    double previous_car_y = car_y - sin(car_yaw);
          	    //add x values (spline requires that points are sorted)
          	    anchor_points_x.push_back(previous_car_x);
          	    anchor_points_x.push_back(car_x);
          	    //add y values (spline requires that points are sorted)
          	    anchor_points_y.push_back(previous_car_y);
          	    anchor_points_y.push_back(car_y);
          	}
          	else //use previous path's end point as starting reference
          	{
          	    //redefine reference state as previous path's end point (for continuity between the previous path and the future path)
          	    reference_x = previous_path_x[previous_path_size - 1];
          	    reference_y = previous_path_y[previous_path_size - 1];
          	    //also get second-to-last previous path point and then compute reference yaw based on them
          	    double previous_reference_x = previous_path_x[previous_path_size - 2];
          	    double previous_reference_y = previous_path_y[previous_path_size - 2];
          	    //compute reference yaw
          	    //reference_yaw = atan2((reference_y - previous_reference_y), (reference_x - previous_reference_x));
          	    //use the two points that make the path tangent to the previous path's end point
          	    //add x values (spline requires that points are sorted)
          	    anchor_points_x.push_back(previous_reference_x);
          	    anchor_points_x.push_back(reference_x);
          	    //add y values (spline requires that points are sorted)
          	    anchor_points_y.push_back(previous_reference_y);
          	    anchor_points_y.push_back(reference_y);
          	}

          	//compute the lane we need to be in based on our target lane defined above
          	//lane width = 4 meters
          	const double lane_width = 4;
          	//we want to stay in the middle of the lane we're in, which is 2 meters in from the lane's edge (or half of the lane width)
          	const double target_location_in_lane = lane_width / 2;
          	//our waypoints are located in the middle of the road at the double yellow line, we always want to be in the center of whichever lane
          	//we happen to be in, so starting in center of the left-most lane (2 meters to the right of the waypoint location) we add our target lane
          	//multiplied by our lane width and this gives us our d value, how many meters from the waypoint we want to be, for example, to be in the middle
          	//lane, we start from the center of the left lane (2 meters to the left of the waypoint) and add (4 * 1), 4 being the lane width and 1 being
          	//our target lane, this means our d value will be 6, we need the center of our vehicle to be 6 meters to the left of the waypoint for us to
          	//be in the center of the middle lane
          	double next_car_d = target_location_in_lane + (lane_width * target_lane);

          	//for our future path, we now plot 3 points into the future and use spline to fill in the gaps
          	//convert the s and d values into an x/y coordinate
          	vector<double> next_xy_30 = getXY(car_s + 30, next_car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_xy_60 = getXY(car_s + 60, next_car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_xy_90 = getXY(car_s + 90, next_car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          	//push the spaced future points onto the vectors
          	//x (spline requires that points are sorted)
          	anchor_points_x.push_back(next_xy_30[0]);
          	anchor_points_x.push_back(next_xy_60[0]);
          	anchor_points_x.push_back(next_xy_90[0]);
          	//y (spline requires that points are sorted)
          	anchor_points_y.push_back(next_xy_30[1]);
          	anchor_points_y.push_back(next_xy_60[1]);
          	anchor_points_y.push_back(next_xy_90[1]);

          	//conduct translation and rotation of all path points to simplify future steps
          	//the points are being transformed to the vehicle's perspective
          	for (int i = 0; i < anchor_points_x.size(); i++)
          	{
          	    //this allows us to center the car at the origin (x = 0, y = 0)
          	    //translate (center the coordinates)
          	    double translated_x = anchor_points_x[i] - reference_x;
          	    double translated_y = anchor_points_y[i] - reference_y;
          	    //this allows us to make yaw zero
          	    //rotate (counter-clockwise, i.e., yaw is negative)
          	    anchor_points_x[i] = (translated_x * cos(-reference_yaw)) - (translated_y * sin(-reference_yaw));
          	    anchor_points_y[i] = (translated_x * sin(-reference_yaw)) + (translated_y * cos(-reference_yaw));
          	}

          	//create a spline
          	tk::spline s;

          	//init spline with anchor points we've derived
          	s.set_points(anchor_points_x, anchor_points_y);

          	//(x, y) values that the planner will use during the next iteration
          	vector<double> next_x_vals;
            vector<double> next_y_vals;

          	//populate all the points left over from the last plan (for continuity between path plans)
          	//this will also keep us from having to generate a whole path each time, we only generate points for amount of points
          	//that were consumed during the last iteration
          	for (int i = 0; i < previous_path_x.size(); i++)
          	{
          	    next_x_vals.push_back(previous_path_x[i]);
          	    next_y_vals.push_back(previous_path_y[i]);
          	}

          	//we now want to ensure that we maintain our target velocity, to do this we need to make sure the points are spaced apart at the appropriate distance
          	//we first look out at some horizon value from the current position of the vehicle (30 meters)
          	double horizon_x = 30.0; //meters
          	double horizon_y = s(horizon_x); //the y value on the line at that particular x value
          	//we then want to split that distance from the vehicle to that horizon value into N pieces along the spline such that we maintain our target velocity
          	//the car will visit a point on our path every 0.02 seconds, so to determine the Euclidean distance between the vehicle and the horizon point,
          	//we compute N * 0.2 * target_velocity = distance (i.e. the hypotenuse)
          	//compute the distance from the horizon point on the spline to the vehicle's current position, remember the vehicle is at zero degrees (yaw) and it's position is at
          	//the origin given our previous coordinate transformation, so no need to subtract the vehicle's position in the distance equation (since it's at zero anyway), we just square the position
          	//of the horizon point at x and y
          	double distance_to_horizon_point = distance(0, 0, horizon_x, horizon_y);
          	//now that we have the the distance, we can compute N (the number of pieces, spaced far enough apart to maintain our target velocity)
          	double N = distance_to_horizon_point / (0.02 * (reference_velocity / 2.24)); //reference velocity is in mph and we need that to be in meters per second so we divide by 2.24
          	//increments x by a value that ensures appropriate spacing to maintain velocity
          	double x_increment = horizon_x / N;

          	//fill in the rest of the points to get us to 50, we already populated the points left over from the last path plan
          	//now we want to generate points on the new path until we get to 50 in total
          	double previous_x = 0;
          	for (int i = 1; i <= (50 - previous_path_x.size()); i++)
          	{
          	    //current x point equals the last x point plus the distance increment that ensures we maintain our target velocity
          	    double x = previous_x + x_increment;
          	    double y = s(x); //give us the y value on the spline for the given x

          	    //store current x point
          	    previous_x = x;

          	    //now we need to transform (translate & rotate) back to global coordinates (as we're still in local coordinates - in the vehicle's perspective)
          	    //rotate (in positive direction)
          	    double rotated_x = (x * cos(reference_yaw)) - (y * sin(reference_yaw));
          	    double rotated_y = (x * sin(reference_yaw)) + (y * cos(reference_yaw));

          	    //translate (adding reference x and y to rotated counterparts) to complete transformation back to global coordinates
          	    //push the new x/y point onto the vectors that go back to the simulator
          	    next_x_vals.push_back(rotated_x + reference_x);
          	    next_y_vals.push_back(rotated_y + reference_y);
          	}

          	/*
          	//go forward and stay in current lane
          	double dist_inc = 0.5;
          	for (int i = 0; i < 50; i++)
          	{
          	    //compute new s & d values
          	    //we add 1 to i as we're starting at zero index and the first next_car_s value would be the same as car_s and we'd fail to move
          	    double next_car_s = car_s + (dist_inc * (i + 1));

          	    //convert the s and d values into an x/y coordinate
          	    vector<double> xy = getXY(next_car_s, next_car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                //push the new x/y point onto the vectors that go back to the simulator
          	    next_x_vals.push_back(xy[0]);
          	    next_y_vals.push_back(xy[1]);
          	}
          	*/

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
