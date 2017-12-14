#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

constexpr double MIN_STEER = -1;
constexpr double MAX_STEER = 1;

constexpr double IDLE_SPEED_CTE       = 10;
constexpr double IDLE_SPEED_THOLD     = 1;
constexpr int    IDLE_SPEED_MAX_TIMES = 10;

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

int idle_speed_cntr = 0;

int main()
{
  uWS::Hub h;

  PID pid;
  pid.Init(1.7341, 0, 6.78751);
  //pid.startTwiddle(1e-2, 1000, 1);


  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double cte   = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          // Check if vehicle is stuck in place from it's speed.
          if (speed < IDLE_SPEED_THOLD) {
            ++idle_speed_cntr;

            // Set the CTE to `IDLE_SPEED_CTE` in order to increase the MSE
            // when vehicle is stuck.
            cte = IDLE_SPEED_CTE;

            // When using twiddle, we want to stop the simulation if vehicle is stuck.
            if (idle_speed_cntr == IDLE_SPEED_MAX_TIMES) {
              pid.simulator_idle_speed = true;
              std::cout << "Idle speed detected!" << std::endl;
            }
          } else if (idle_speed_cntr) {
            idle_speed_cntr = 0;
          }

          pid.UpdateError(cte);

          // Check if PID sent reset request to simulator.
          if (pid.simulator_reset_request) {
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            // reset idle speed params
            pid.simulator_idle_speed = false;
            idle_speed_cntr = 0;

            pid.simulator_reset_request = false;
          } else {
            steer_value = -pid.TotalError();

            /* Normalize steering value to [-1, 1] by trimming
            * values outside of limits. */
            if (steer_value < MIN_STEER) {
              steer_value = MIN_STEER;
            } else if (steer_value > MAX_STEER) {
              steer_value = MAX_STEER;
            }

            // DEBUG
            //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
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
