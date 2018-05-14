#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <cmath>
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

int mod3Inc(int current) {
  ++current;
  return (current == 3)?0:current;
}

double sum(double p[]) {
  double s = 0.0;
  for (int i = 0; i < 3; i++) {
    s += p[i];
  }
  return s;
}

int main()
{
  uWS::Hub h;

  PID pid;
  //double p[] =  {0.05, 0.0001, 0.001};
  //double p[] =  {1.08843,0.000107921,0.0507921};
  //double p[] =  {2.18843,0.00115792,0.100792};
  //double p[] =  {1.06843,0.00220792,0.00105792};
  //double p[] =  {2.24843,0.0010792,0.0105792};
  //double p[] =  {2.5784,0.0010792,0.8105792};
  //double p[] = {0.14418,0.0001422,2.26171};
  // double p[] = {0.04418,0.0001322,2.25171};
  //double p[] =  {1.02, 0.0142, 2.331};
  //double p[] =  {1.58843,0.00112921,0.022421};
  double p[] = {0.15538,0.0001212,2.30171};
  // Following also needs a change if parameters are learned using Twiddle
  double dp[] = {0.001,0.0001,0.1};
  p[0] += dp[0];
  pid.Init(p[0], p[1], p[2]);
  int curTotal = 0;
  int currentParam = 0;
  // With smaller threshold we start using Twiddle. Choosing a large threshold so
  // that we can see the effect of the car driving with the final parameters.
  int threshHold = 1000000;
  double cteErr = 0.0;
  double bestErr = 1.0;
  bool prevCheck = false;
  double alpha = 1.0;

  h.onMessage([&alpha, &pid, &curTotal, &currentParam, &threshHold, &cteErr, &bestErr, &prevCheck, &dp, &p](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          double throttle;
          throttle = 0.4 - alpha * std::abs(steer_value);
          throttle = (throttle < 0.1)?0.1:throttle;

          
          // DEBUG
          cteErr += cte * cte;
          curTotal++;
          if (curTotal > threshHold) {
            // Change the parameter
            cteErr = cteErr/threshHold;
            if (prevCheck) {
              if (cteErr < bestErr) {
                bestErr = cteErr;
                dp[currentParam] *= 1.1;
              } else {
                p[currentParam] += dp[currentParam];
                dp[currentParam] *= 0.9;
              }
              currentParam = mod3Inc(currentParam);
              prevCheck = false;
            }
            else if (cteErr < bestErr) {
              bestErr = cteErr;
              dp[currentParam] *= 1.1;
              currentParam = mod3Inc(currentParam);
              p[currentParam] += dp[currentParam];
            } else {
              p[currentParam] -= 2*dp[currentParam];
              prevCheck = true;
            }
            curTotal = 0;
            pid.Init(p[0], p[1], p[2]);
            cteErr = 0.0;
          }
          
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;

          msgJson["steering_angle"] = steer_value;
          //msgJson["throttle"] = (1 - std::abs(steer_value)) * 0.5 + 0.2;
          msgJson["throttle"] = throttle;
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
