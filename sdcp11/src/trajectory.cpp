/*
#######################################################
## AUTHOR: James Beasley                             ##
## DATE: January 28, 2018                            ##
## UDACITY SDC: Project 11 (Path Planning)           ##
#######################################################
*/

//includes
#include "trajectory.h"

//class definition
//constructor
Trajectory::Trajectory() {}

//destructor
Trajectory::~Trajectory() {}

//function definition
//based on the behavioral guidance, compute a vehicle trajectory
vector<vector<double>> Trajectory::compute_vehicle_path(const VehicleTelemetry& vehicle_telemetry, const vector<Waypoint>& map_waypoints)
{
    //local vars

    //indicates the lane we want to be in (left = 0, middle = 1, right = 2)
    int target_lane = 1;
    //define a reference velocity that acts as the speed we want to hover around but not exceed
    double reference_velocity = 49.5;
    //capture the size of the previous set of path points that were not reached in the last iteration (the remaining points at the end of the previous plan)
    int previous_path_size = vehicle_telemetry.unconsumed_previous_path_x.size();

    //create a list of widely spaced (x, y) anchor points, evenly spaced apart at 30 meters
    //we'll interpolate these points using the spline library to fill in the gaps at the spacing
    //we need to maintain our reference velocity (remember the spacing determines acceleration)
    vector<double> anchor_points_x;
    vector<double> anchor_points_y;

    //establish reference x, y, yaw states
    //on the first iteration we'll not have a previous set of path points to extend upon so we need to start with the vehicle's current location
    double reference_x = vehicle_telemetry.x;
    double reference_y = vehicle_telemetry.y;
    double reference_yaw = utility_.convert_from_degrees_to_radians(vehicle_telemetry.yaw);

    //if the previous path size contains less then 2 points, we'll need to use the vehicle's position as a starting reference
    //we'll create a second point (previous to the vehicle's current position) based on the vehicle's current yaw
    if (previous_path_size < 2)
    {
        //compute two anchor points (the vehicle's current position and a point before that) to use since we don't have a previous path to look at
        //we use the vehicle's yaw angle to ensure the previous point is on the same line as the vehicle's angle
        double previous_car_x = vehicle_telemetry.x - cos(vehicle_telemetry.yaw);
        double previous_car_y = vehicle_telemetry.y - sin(vehicle_telemetry.yaw);
        //add x values (spline requires that points are sorted)
        anchor_points_x.push_back(previous_car_x);
        anchor_points_x.push_back(vehicle_telemetry.x);
        //add y values (spline requires that points are sorted)
        anchor_points_y.push_back(previous_car_y);
        anchor_points_y.push_back(vehicle_telemetry.y);
    }
    else //use previous path's end point as starting reference
    {
        //redefine reference state as previous path's end point (for continuity between the previous path and the future path)
        reference_x = vehicle_telemetry.unconsumed_previous_path_x[previous_path_size - 1];
        reference_y = vehicle_telemetry.unconsumed_previous_path_y[previous_path_size - 1];
        //also get second-to-last previous path point and then compute reference yaw based on them
        double previous_reference_x = vehicle_telemetry.unconsumed_previous_path_x[previous_path_size - 2];
        double previous_reference_y = vehicle_telemetry.unconsumed_previous_path_y[previous_path_size - 2];
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
    vector<double> next_xy_30 = utility_.convert_from_frenet_to_cartesian(vehicle_telemetry.s + 30, next_car_d, map_waypoints);
    vector<double> next_xy_60 = utility_.convert_from_frenet_to_cartesian(vehicle_telemetry.s + 60, next_car_d, map_waypoints);
    vector<double> next_xy_90 = utility_.convert_from_frenet_to_cartesian(vehicle_telemetry.s + 90, next_car_d, map_waypoints);

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
    for (int i = 0; i < vehicle_telemetry.unconsumed_previous_path_x.size(); i++)
    {
        next_x_vals.push_back(vehicle_telemetry.unconsumed_previous_path_x[i]);
        next_y_vals.push_back(vehicle_telemetry.unconsumed_previous_path_y[i]);
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
    double distance_to_horizon_point = utility_.compute_distance_between_points(0, 0, horizon_x, horizon_y);
    //now that we have the the distance, we can compute N (the number of pieces, spaced far enough apart to maintain our target velocity)
    double N = distance_to_horizon_point / (0.02 * (reference_velocity / 2.24)); //reference velocity is in mph and we need that to be in meters per second so we divide by 2.24
    //increments x by a value that ensures appropriate spacing to maintain velocity
    double x_increment = horizon_x / N;

    //fill in the rest of the points to get us to 50, we already populated the points left over from the last path plan
    //now we want to generate points on the new path until we get to 50 in total
    double previous_x = 0;
    for (int i = 1; i <= (50 - vehicle_telemetry.unconsumed_previous_path_x.size()); i++)
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

    return {next_x_vals, next_y_vals};
}
