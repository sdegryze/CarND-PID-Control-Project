#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;
  
  // set to true if you want to optimize the PID coefficients using twiddle
  // if set to false, the initial values specified below will be used as (constant) PID coefficients
  bool run_twiddle = false;

  // a PID controller for the steering angle
  PID steering_pid;
  
  // a PID controller for the throttle
  PID throttle_pid;
  
  // an instance of the Twiddle object to conduct parameter optimization
  Twiddle twiddle;

  // The coefficient values for the steering angle have been tuned using twiddle
  steering_pid.Init(0.191304, 0.000277616, 4.61365); // Kp, Ki, Kd
  twiddle.Init(0.191304, 0.000277616, 4.61365);
  
  // The coefficient values for the throttle were manually tuned
  throttle_pid.Init(0.4, 0., 2.0);
  

  h.onMessage([&steering_pid, &throttle_pid, &twiddle, &run_twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle_value;

          steering_pid.UpdateError(cte);
          throttle_pid.UpdateError(fabs(cte));
          
          // clip steering value between -1 and 1
          steer_value = max(min(steering_pid.TotalError(), 1.), -1.);
          // clip throttle value between -0.3 (breaking) and 0.9
          throttle_value = max(min(0.65 + throttle_pid.TotalError(), 0.9), -0.3);
          
          // uncomment for debugging:
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          // std::cout << "CTE: " << cte << " Throttle Value: " << throttle_value << std::endl;
          
          if (run_twiddle) {
            bool reset_needed = twiddle.Step(cte);
            if (reset_needed) {
              vector<double> coefs = twiddle.GetUpdatedCoefs();
              steering_pid.Init(coefs[0], coefs[1], coefs[2]);
              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
