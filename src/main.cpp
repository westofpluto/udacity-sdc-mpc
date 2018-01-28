#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double d2r25 = deg2rad(25.0);

double latency = 0.1;    

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}


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
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          if (psi > pi()) {
              psi -=2*pi();
          } else if (psi < -pi()) {
              psi += 2*pi();
          }
          //double ptdy=ptsy[1]-ptsy[0]; 
          //double ptdx=ptsx[1]-ptsx[0]; 
          //double psi_road = atan2(ptdy,ptdx);

          std::cout << "PSI: " << psi << "\n";
          //std::cout << "PSI_ROAD: " << psi_road << "\n";
          //
          // steering angle in sumulator is normalized such that
          // 1.0 = 25 deg
          // -1.0 = -25 deg
          //
          double delta_norm = j[1]["steering_angle"];
          double delta_rad = d2r25*delta_norm;
          double a = j[1]["throttle"];

          //
          // begin by calculating trajectory errors in car frame of reference
          // This yields the vectors err_x and err_y which are the desired trajectory waypoints in the car frame of reference
          //
          vector<double> err_x;
          vector<double> err_y;
          int npts=ptsx.size();
          double cpsi=cos(psi);
          double spsi=sin(psi);

          for (int i=0;i<npts;i++) {
              double dx=ptsx[i] - px;
              double dy=ptsy[i] - py;
              //
              // dx and dy are errors in world frame. rotate these into car frame
              //
              double dx_car=dx*cpsi + dy*spsi;
              double dy_car=-dx*spsi + dy*cpsi;
              err_x.push_back(dx_car);
              err_y.push_back(dy_car);
          }

          //
          // Now use err_x and err_y in polyfit
          //
          double* p_err_x = &err_x[0];
          double* p_err_y = &err_y[0];
          int num_waypts=6;
          Eigen::Map<Eigen::VectorXd> waypoints_x_car(p_err_x, num_waypts);
          Eigen::Map<Eigen::VectorXd> waypoints_y_car(p_err_y, num_waypts);

          //
          // fit points to a 3rd order polynomial. Again these are in car coordinates, not world coords.
          // In car coordinates, x and y and psi are all 0.0
          //
          auto coeffs = polyfit(waypoints_x_car, waypoints_y_car, 3);

          //
          // cte is defined as reference y - actual y (here in car frame)
          // But in car frame y = 0, so cte = reference y - 0.0 = polyeval(coeffs,0.0)
          //
          // cte is defined in this problem as refy - actual y
          //
          double cte = polyeval(coeffs, 0.0);  
          double deriv = polyderiveval(coeffs, 0.0);
          //
          // epsi is defined as psi - psidesired where psidesired=atan(deriv)
          // But we are in car coordinates and in car coordinates psi is 0.0
          //
          double epsi = 0.0 -atan(deriv);  

          /*
          * Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value;
          double throttle_value;

          //
          // set the state variables in the car frame (or more accurately, in an inertial frame that is
          // exacty aligned with the car at the current time but is fixed as time evolves)
          // We must also handle latency so lets assume the initial state in the optimization is
          // actually the cars current state propagated forward one time step
          //
          double Lf = mpc.get_Lf();

          double x0 = 0.0;
          double y0 = 0.0;
          double psi0 = 0.0;
          double v0 = v; 
          double cte0 = cte;
          double epsi0 = epsi;

          //
          // propagate forward in time by dt=latency
          //
          double x1 = x0 + v0*cos(psi0)*latency;
          double y1 = y0 + v0*sin(psi0)*latency;
          double psi1 = psi0 - (v0/Lf)*delta_rad*latency;
          double v1 = v0 + a * latency;
          double cte1 = cte0 + v0*sin(epsi0)*latency;
          double epsi1 = epsi0 - (v0/Lf)*delta_rad*latency ;

          Eigen::VectorXd initial_state(6);
          initial_state << x1, y1, psi1, v1, cte1, epsi1;

          std::cout << "cte is " << cte << ", and epsi is " << epsi << std::endl;

          auto start_ts = std::chrono::system_clock::now();
          auto optvars = mpc.Solve(initial_state, coeffs);
          auto end_ts = std::chrono::system_clock::now();
          std::chrono::duration<double> elapsed_time = end_ts-start_ts;
          double telapsed=elapsed_time.count();
          std::cout << "ELAPSED TIME IS " << telapsed << std::endl;

          steer_value = optvars[0];
          throttle_value = optvars[1];

          std::cout << "CONTROLS: steering is " << (steer_value/d2r25) << ", and throttle is " << throttle_value << std::endl;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          // This is because the simulator expects a normalized steering angle from -1 to 1 where
          // +/- 1 is the same as +/- 25 deg

          msgJson["steering_angle"] = (steer_value/d2r25);

          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          for (int i = 2; i < optvars.size(); i ++) {
              if (i%2 == 0) {
                  mpc_x_vals.push_back(optvars[i]);
              } else {
                  mpc_y_vals.push_back(optvars[i]);
              }
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          /*
          double vxunit = cos(psi);
          double vyunit = sin(psi);
          double desx=ptsx[1]-ptsx[0]; 
          double desy=ptsy[1]-ptsy[0];
          double desmag = sqrt(desx+desx + desy*desy);
          double cth=(vxunit*desx + vyunit*desy)/desmag;
          double sth = (-vxunit*desy + vyunit*desx)/desmag;
          double theta=atan2(sth,cth);
          */

          //
          // rotate to road coords
          //
          //double psi_road = atan2(ptdy,ptdx);
          double cth=1.0;
          double sth=0.0;
          for (int i = 0; i < 100; i += 3){
              double xtmp = (double)i;
              double ytmp = polyeval(coeffs, xtmp);
              double xx = cth * xtmp +sth*ytmp;
              double yy = -sth * xtmp + cth*ytmp;

              next_x_vals.push_back(xx);
              next_y_vals.push_back(yy);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
