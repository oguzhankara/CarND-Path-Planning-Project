#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  // Starts in Lane 1
  int lane = 1;
    
  // Reference velocity to target
  double velocity = 0.0; // mph
  double change_velocity_by = .224;
  const double top_velocity = 49.5;

  
  h.onMessage([&top_velocity, &change_velocity_by, &velocity, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          /* * * * * * * * * * * * * * * * * * Prediction Component * * * * * * * * * * * * * * * * * */
          //  Estimates the actions of other objects from the sensory input for the near future
          //  Most of the trajectories are assumed to be straight lines for other objects
          /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
          int prev_path_size = previous_path_x.size();
          
          if(prev_path_size > 0)
          {
            car_s = end_path_s;
          }
          
          bool sensed_car_left= false;
          bool sensed_car_right = false;
          bool sensed_car_front = false;
          
          for(int i=0; i < sensor_fusion.size(); i++)
          {
            float d = sensor_fusion[i][6];
            int sensed_car_lane;
            /* Each lane is assumed to be 4 meter in width */
            // There are 3 lanes in total. Leftmost lane ID is 0, middle lane ID is 1 and rightmost lane ID is 2
            if(d >= 0 && d <= 4){sensed_car_lane = 0;} 
            else if(d > 4 && d <= 8) {sensed_car_lane = 1;} 
            else if(d > 8 && d <= 12) {sensed_car_lane = 2;}
            
            double sensed_velocity_x = sensor_fusion[i][3];
            double sensed_velocity_y = sensor_fusion[i][4];
            double sensed_car_velocity = sqrt((sensed_velocity_x*sensed_velocity_x)+(sensed_velocity_y*sensed_velocity_y));
            double sensed_car_s = sensor_fusion[i][5];
            
            // This will help to predict the where the vehicle will be in future
            sensed_car_s += ((double)prev_path_size*0.02*sensed_car_velocity);
            
            if(sensed_car_lane == lane) // A vehicle is on the same line and check the car is in front of the ego car
            {sensed_car_front |= sensed_car_s > car_s && (sensed_car_s - car_s) < 30;}
            else if((sensed_car_lane - lane) == -1) // A vehicle is on the left lane and check whether it is in 30 meter (front) 20 meter (backward) range
            {sensed_car_left |= (car_s + 30) > sensed_car_s  && (car_s - 20) < sensed_car_s;}
            else if((sensed_car_lane - lane) == 1)// A vehicle is on the right lane and check whether it is in 30 meter (front) 20 meter (backward) range
            {sensed_car_right |= (car_s + 30) > sensed_car_s  && (car_s - 20) < sensed_car_s;}
          } 
          
          /* * * * * * * * * * * * * * * * * * Behaviour Component * * * * * * * * * * * * * * * * * */
          //  Determines the actions of Ego car while driving in the highway.
          /*  These actions are;
                >> The car drives according to the speed limit.
                >> Max Acceleration and Jerk are not Exceeded.
                >> Car does not have collisions by changing lanes or decreasing speed.
          */
          /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */          
          if(sensed_car_front)
          {
            if(lane==0 && !sensed_car_right){lane++;}
            else if(lane==1 && !sensed_car_left){lane--;}
            else if(lane==1 && !sensed_car_right){lane++;}
            else if(lane==2 && !sensed_car_left){lane--;}
            else {velocity-=change_velocity_by;}
          }
          else if(velocity < top_velocity) {velocity += (1.2 * change_velocity_by);}
          
          
          /* * * * * * * * * * * * * * * * * * Trajectory Planning * * * * * * * * * * * * * * * * * */
          //  Calculates the trajectory based on the speed and lane output from the behavior component, 
          //  car coordinates and past path points.
          /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */             
          
          vector<double> ptsx;
          vector<double> ptsy;
          
          //Refrence x,y, and yaw states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // If previous states are almost empty, use the car as a starting point
          if (prev_path_size < 2)
          {
            //Use two points thats makes path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
          {
            //Redefine the reference point to previous point
            ref_x = previous_path_x[prev_path_size - 1];
            ref_y = previous_path_y[prev_path_size - 1];
            double ref_x_prev = previous_path_x[prev_path_size - 2];
            double ref_y_prev = previous_path_y[prev_path_size - 2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          // Setting up target points in the future.
          vector<double> next_wp0 = getXY(car_s + 30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          // Making coordinates to local car coordinates.
          for (int i = 0; i < ptsx.size(); i++)
          {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }
          
          // Create the spline.
          tk::spline s;
          s.set_points(ptsx, ptsy);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          //For the smooth transition, we are adding previous path points
          for (int i=0; i<prev_path_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Calculate distance y position on 30 m ahead.
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          double x_add_on = 0;
          
          for(int i = 1; i < 50 - prev_path_size; i++)
          {
            double N = target_dist/(0.02*velocity/2.24);
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);
            x_add_on = x_point;
            double x_ref = x_point;
            double y_ref = y_point;
            
            //Rotate back to normal after rotating it earlier
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
            x_point += ref_x;
            y_point += ref_y;
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          json msgJson;       
  

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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