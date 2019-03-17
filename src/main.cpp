#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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
  int lane = 1;
  double ref_vel = 0.0;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&ref_vel,&lane]
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

          json msgJson;

           /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          int prev_size = previous_path_x.size();

          if(prev_size > 0)
          {
            car_s = end_path_s;
          }
          
          double ego_buffer = 30;
          bool too_close = false;
          bool change_left = true;
          bool change_right = true;
          vector<double> closest_s = {1000.0, 1000.0};

          // Process sensor fusion data
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
            float other_d = sensor_fusion[i][6];

            // Ego lane processing to see if the car ahead is too close, any lane change is prompted by only this
            if(other_d < (2+4*lane+2) && other_d > (2+4*lane-2))
            {
              double other_vx = sensor_fusion[i][3];
              double other_vy = sensor_fusion[i][4];
              double other_speed = distance(0, 0, other_vx, other_vy);
              double other_s = sensor_fusion[i][5];

              other_s += ((double)prev_size * 0.02 * other_speed);

              if((other_s > car_s) && (other_s - car_s < ego_buffer)) // should I check for both?
              {
                too_close = true;
              }
            }

            // Process relative lanes, left and right

            // Left lane
            if(other_d < (2+4*(lane-1)+2) && other_d > (2+4*(lane-1)-2))
            {
              double left_lane_vx = sensor_fusion[i][3];
              double left_lane_vy = sensor_fusion[i][4];
              double left_lane_speed = distance(0, 0, left_lane_vx, left_lane_vy);
              double left_lane_s = sensor_fusion[i][5];
              
              left_lane_s += ((double)prev_size * 0.02 * left_lane_speed);
              
              if(std::fabs(left_lane_s - car_s) < ego_buffer) // should I check for both?
              {
                if (std::fabs(left_lane_s - car_s) < closest_s[0])
                {
                  closest_s[0] = std::fabs(left_lane_s - car_s);
                }
                change_left = false;
              }
            }

            // Right lane
            if(other_d < (2+4*(lane+1)+2) && other_d > (2+4*(lane+1)-2))
            {
              double right_lane_vx = sensor_fusion[i][3];
              double right_lane_vy = sensor_fusion[i][4];
              double right_lane_speed = distance(0, 0, right_lane_vx, right_lane_vy);
              double right_lane_s = sensor_fusion[i][5];
              
              right_lane_s += ((double)prev_size * 0.02 * right_lane_speed);
              
              if(std::fabs(right_lane_s - car_s) < ego_buffer) // should I check for both?
              {
                if (std::fabs(right_lane_s - car_s) < closest_s[1])
                {
                  closest_s[1] = std::fabs(right_lane_s - car_s);
                }
                change_right = false;
              }
            }
          }
          
          std::cout << "Closest left: " << closest_s[0] << "  Closest right: " << closest_s[1] << std::endl;
          std::cout << "Change  left: " << change_left  << "  Change  right: " << change_right << std::endl;
          std::cout << "Too close:    " << too_close << std::endl;
         
          if(too_close)
          {
            
            // Slow down in lane
            if (ref_vel > 0)
            {
              ref_vel -= 0.224;
            }

            if(change_left && lane > 0)
            {
              lane -= 1;
            }
            else if(change_right && lane < 2)
            {
              lane += 1;
            }

          }
          else
          {
            // std::cout << "Current lane: " << lane << ", Fastest lane: " << fastest_lane << std::endl;
            // std::cout << "Keep lane, in the fastest lane" << std::endl;    
            // Acclerate to speed limit
            if (ref_vel <49.5)
            {
              ref_vel += 0.224;
            }      
          }


          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = car_yaw;

          // Make up coarse set of way points ptsx, ptsy to construct spline from later

          // Check if previous path exists, if it is small, calculate last 2 points,
          // If not, just assign last 2 points. And calculate car yaw from them.
          if (prev_size < 2)
          {
            double prev_x = car_x - cos(car_yaw);
            double prev_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_y);
            ptsy.push_back(car_y);
          }

          else
          {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
          }

          // Add next set of coarse points evenly spaced 30m ahead of the car.
          vector<double> next_wp0 = getXY(car_s + 30, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // Convert X,Y from global to local coordinate system for easy calculations.
          // This makes reference angle 0.
          for(int i= 0; i < ptsx.size(); i++)
          {
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0 - ref_yaw));

          }

          // Now for the actual points for the planner...
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all the left over points from the previous run..
          // But why?.. We have already accounted for these points in the spline coarse points..
          for(int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Create spline and set constructed coarse points from previous path and evenly spaced points.
          tk::spline s;
          s.set_points(ptsx,ptsy);

          // Calculate interval for spline points so we travel at desired velocity
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = distance(0,0,target_x,target_y);

          double x_add_on = 0;

          for(int i = 1; i <= 50 - previous_path_x.size(); i++)
          {
            double N = (target_dist/(0.02*(ref_vel/2.24)));
            double x_point = x_add_on + (target_x/N);
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw)); 

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }


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