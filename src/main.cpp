#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "MPC.hpp"
#include "json.hpp"

#include "utilities.hpp"

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}



int main() {

    // We set the number of timesteps to 25
  // and the timestep evaluation frequency or evaluation
  // period to 0.05.
  const int N = 10;
  const double dt = 0.2;
  const double ref_v_meterspersecond = mph2mps(60.0);

  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc,&N,&dt,&ref_v_meterspersecond](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);

    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object

          // xy points along the desired trajectory in global frame
          vector<double> ptsx_vec = j[1]["ptsx"];
          vector<double> ptsy_vec = j[1]["ptsy"];

          // convert to eigen vectors
          const int numpts = ptsx_vec.size();
          Eigen::Map<Eigen::VectorXd> ptsx(ptsx_vec.data(),numpts);
          Eigen::Map<Eigen::VectorXd> ptsy(ptsy_vec.data(),numpts);

          // current state
          const double px = j[1]["x"];
          const double py = j[1]["y"];
          const double psi = j[1]["psi"];
          const double current_speed_mps = mph2mps(j[1]["speed"]);

          // convert desired tracjectory to vehicle frame
          Eigen::VectorXd ptsx_veh;
          Eigen::VectorXd ptsy_veh;
          std::tie(ptsx_veh,ptsy_veh) = global2veh(px,py,psi,ptsx,ptsy);

          // fit desired trajectory with cubic polynomial
          const Eigen::VectorXd poly_coeffs = polyfit(ptsx_veh,ptsy_veh,3);

          // cross track error is first coefficient, f(0)
          const double cte = poly_coeffs[0];

          // heading error is -f'(0)
          const double epsi = -atan(poly_coeffs[1]);

          // current state in vehicle frame
          Eigen::VectorXd state(6);
          state << 0.0,0.0,0.0,current_speed_mps,cte,epsi;

          // call the solver
          vector<double> solutionx = mpc.Solve(state,poly_coeffs,N,dt,ref_v_meterspersecond);

          //grab the coordinates of the controlled trajectory for plotting
          vector<double>::const_iterator first;
          vector<double>::const_iterator last;
          first = solutionx.begin();
          last = solutionx.begin() + N;
          vector<double> solx(first, last);
          first = solutionx.begin() + N;
          last = solutionx.begin() + 2*N;
          vector<double> soly(first, last);

          // also grab the currently recommended steer and throttle values
          const double steer_value = solutionx[6*N];
          const double throttle_value = solutionx[7*N-1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = (steer_value/deg2rad(25.0));
          msgJson["throttle"] = throttle_value;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = solx;
          msgJson["mpc_y"] = soly;

          //Display the waypoints/reference line
          vector<double> next_x_vals(ptsx_veh.data(),ptsx_veh.data() + ptsx_veh.size());
          vector<double> next_y_vals(ptsy_veh.data(),ptsy_veh.data() + ptsy_veh.size());

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does not actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(0));
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
