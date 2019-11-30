#include <uWS/uWS.h>
#include <fstream>
#include <math.h>
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

// start in lane 1
int lane = 1;
// have a referecne velocity to target
double ref_vel = 0.0;//mph
double slowfactor = 1.0; // 
double gap_th = 30.0;
double gapfactor = 1.0;
double diffvfacotr = 40.0;


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

  

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // added following the video
          int prev_size = previous_path_x.size();

          /* considering sensor fusion data*/
          if(prev_size > 0)
          {
            car_s = end_path_s;
          }
         
          // std::cout<< "ref_vel =" << ref_vel << std::endl;  
          bool too_close = false;
          bool RCmerge = false;
          bool LCmerge = false;

          
          //find ref_v to ruse considering other car i
          double FrontCarSpeed = 49.5;
          double cost_left = 0.1; // bias for left turn, to break the symmetry if cost of left and right is the same.
          double cost_right = 0.0;
          double cost_keep = 0.0;

          double old_v = 49.5;

          for(int i = 0; i< sensor_fusion.size(); i++) {
            double other_x = sensor_fusion[i][1];
            double other_y = sensor_fusion[i][2];
            double other_vx = sensor_fusion[i][3];
            double other_vy = sensor_fusion[i][4];
            double other_speed = sqrt(other_vx*other_vx + other_vy*other_vy);
            double other_s = sensor_fusion[i][5];
            other_s += ((double)prev_size*.02*other_speed);// if using previous points can project s value out
            float other_d = sensor_fusion[i][6];
            
            double gap_s = other_s - car_s; 
            double gap_d = other_d - car_d;

            bool same_lane = (other_d < (2 + 4 * lane + 2) && other_d > (2 + 4 *lane - 2));
            bool left_lane= (other_d < (2 + 4 * (lane-1) + 2) && other_d > (2 + 4 *(lane-1) - 2));            
            bool right_lane = (other_d < (2 + 4 * (lane+1) + 2) && other_d > (2 + 4 *(lane+1) - 2));
            
            
            // if car is in the same lane
            if(same_lane) {
              
              if(gap_s > 0 && gap_s <= 30) { // is it close
                
                too_close = true;
                FrontCarSpeed = other_speed;
                // put penalty on staying on the same lane if the front car is too slow
                cost_keep += 49.5/FrontCarSpeed * slowfactor; 
                std::cout<< "Too Close!! FrontCarSpeed =" << FrontCarSpeed << std::endl;
                std::cout<< "Current Lane = " << lane << std::endl;  

              }
 
            }
            
            
            if(right_lane) {// look right 
                  double RightCarSpeed = other_speed;
                  cost_right  += gapfactor*gap_th/abs(gap_s) + (FrontCarSpeed - RightCarSpeed)/diffvfacotr;   
                  // is right lane car ahead merging to my lane?
                  if(gap_s > 0) { 
                  bool RCmerge = abs(gap_d)<3.0 ;
                  }            
            }
            
            else if(left_lane) { // look left
                  double LeftCarSpeed = other_speed;
                  cost_left  += gapfactor*gap_th/abs(gap_s) + (FrontCarSpeed - LeftCarSpeed)/diffvfacotr;
                   // is left lane car ahead merging to my lane?
                  if(gap_s > 0) {
                  bool LCmerge = abs(gap_d)<3.0 ;  
                  }            
            }

          } // loop over sensor fusion


          // Behavior: adjust speed
          if((too_close)  && (ref_vel > FrontCarSpeed)) {
            ref_vel -= .224;            
          }
          else if(RCmerge || LCmerge){ //car on left or right merging into my lane
            ref_vel -= .224;
          }          
          else if(ref_vel < 49.5){
            ref_vel += .224;
          }

          // Behavior : change lane or keep lane
          if(cost_right > cost_left && cost_keep > cost_left && lane>0) {
            lane -= 1;
            std::cout<< "Change to Left. Lane = " << lane << std::endl;
          }
          else if(cost_keep > cost_left && lane == 2) {
            lane -= 1;
            std::cout<< "Change to Left. Lane = " << lane << std::endl;
          }
          else if(cost_left > cost_right && cost_keep > cost_right && lane<2) {
            lane += 1;
            std::cout<< "Change to Right. Lane = " << lane << std::endl;
          }
          else if(cost_keep > cost_right && lane == 0) {
            lane += 1;
            std::cout<< "Change to Right. Lane = " << lane << std::endl;
          
          }


          // for debugging
          if(abs(ref_vel - old_v) >0.2 ) {
            std::cout<< "MyCar Speed = " << ref_vel << std::endl;
            std::cout<< "cost_left = " << cost_left << std::endl;
            std::cout<< "cost_right = " << cost_right << std::endl;
            std::cout<< "cost_keep = " << cost_keep << std::endl;

            std::cout<< "Lane = " << lane << std::endl;
            std::cout<< "==================== " << std::endl;

            old_v = ref_vel;
          }
          // for debugging


          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if(prev_size < 2) {
            // use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else {
            // redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

          }


          // // In frenet add evenly 30m spaced points ahead of the starting reference, they are used as anchor for 
          // //spline the future trajectory follwing the lane line 
          vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
             // total 2+3 = 5 points
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i=0; i<ptsx.size(); i++) {
            //shift car reference angle to 0 degrees (trasform to car's coordinate)
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
            ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);

          }
          // for debugging
          // std::cout << "ptsx:"<<std::endl;
          // for (int i=0; i<ptsx.size(); i++) {
          //   std::cout << ptsx[i] << " ";
          // }
          // std::cout << std::endl;
          // std::cout << "ptsy:"<<std::endl;
          // for (int i=0; i<ptsx.size(); i++) {
          //   std::cout << ptsy[i] << " ";
          // }
          // std::cout << std::endl;
          

          // create a spline
          tk::spline spline_car;

          // set (x,y) points (anchor) to the spline
          spline_car.set_points(ptsx,ptsy);

          // define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // start with all of the previous path points from last time
          for(int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Caculate how to break up spline points so that we travel at our desired velocity
          double target_x = 30.0;
          double target_y = spline_car(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          double x_add_on = 0;

          // Fill up the rest of our path planner after filling it with previous points, here we will always ouput 50 points
          for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

            double N = target_dist/(0.02*ref_vel/2.24); // convert to meter/sec
            double x_point = x_add_on + (target_x)/N;
            double y_point = spline_car(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // coordinate transformation: rotate back to normal after rotatin it earlier
            x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
            y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);
            // coordinate transformation: shfit
            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);


            // for debugging
            // std::cout<<"spline" << std::endl;
            // std::cout<<x_point << ','<< y_point << std::endl;

          }

          /* first try: straight line */
          // // double dist_inc = 0.5;
          // // for (int i = 0; i < 50; ++i) {
          // //   next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
          // //   next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
          // // }
          /*first try in straight line */

          /*second try: stay in a lane */
          // vector<double> next_x_vals;
          // vector<double> next_y_vals;
          // double dist_inc = 0.5;
          // for (int i = 0; i < 50; ++i) {
          //   double next_s = car_s + (i+1)*dist_inc;
          //   double next_d = 6; // to stay in a constant lane
          //   vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          //   next_x_vals.push_back(xy[0]); 
          //   next_y_vals.push_back(xy[1]);
          // }
          /*stay in a lane*/

          json msgJson;
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