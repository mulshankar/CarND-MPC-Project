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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,int order) {
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
          vector<double> ptsx = j[1]["ptsx"]; // way points 'x' from the simulator - size 6
          vector<double> ptsy = j[1]["ptsy"]; // way points 'y' from the simulator - size 6
          double px = j[1]["x"]; // current car position 
          double py = j[1]["y"]; // current car position 
          double psi = j[1]["psi"]; // car heading
          double v = j[1]["speed"]; // car velocity
		  double steer_value=j[1]["steering_angle"];// steer angle reported by simulator
		  double accel=j[1]["throttle"];// throttle reported by simulator
          		  
		  for (int i=0;i<ptsx.size();i++) { // Transform way points from map coordinate to car coordinate system
			
			double shift_x = ptsx[i]-px; // translation move
			double shift_y = ptsy[i]-py;

			ptsx[i] = (shift_x*cos(0-psi) - shift_y*sin(0-psi)); // rotational move
			ptsy[i] = (shift_x*sin(0-psi) + shift_y*cos(0-psi));  		  
		  }
		  
		  double* ptrx=&ptsx[0]; // converting the vector of doubles to Eigen pointers to be used in polyfit
		  Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx,6);
		  
		  double* ptry=&ptsy[0]; // converting the vector of doubles to Eigen pointers to be used in polyfit
		  Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry,6);
		  
		  auto coeffs=polyfit(ptsx_transform,ptsy_transform,3); // fit the way-points to a third order polynomial
		  
		  v=v*0.44704; //mps conversion for solver?
		  
		  double Lf=2.67; // center of gravity of vehicle to front axle.. a vehicle constant provided by Udacity
		  double latency=0.1; //seconds
		    
		  /* LATENCY LOGIC INTRODUCED HERE 		  
		  x_t+1=xt+v*cos(psi)*dt;
		  y_t+1=yt+v*sin(psi)*dt;
		  psi_t+1=psi+(v/Lf)*delta*dt
		  v_t+1=v+a*t;
		  cte_t+1=f(x)-y+v*sin(epsi)*dt; // where epsi=psi-psi_des
		  epsi_t+1=psi-psi_des+(v/Lf)*delta*dt;
		  
		  * Before calculating delta and accel via MPC, predict states at latency dt		  
		  * We are already in the car coordinate system.. x,y and psi are 0 */
		  
		  double x_l=0+v*cos(0)*latency;
		  double y_l=0+v*sin(0)*latency;
		  double psi_l=0+(v/Lf)*(steer_value)*latency;
		  double v_l=v+accel*latency;
		  
		  double epsi=0-atan(coeffs[1]); // psi - psi_des at current time step x=0 in -atan(3*coeffs[2]*x^2+2*coeffs[2]*x+coeffs[1])	  
		  double cte_l=polyeval(coeffs,0)-0+v*sin(epsi)*latency;
		  
		  double epsi_l=0-atan(coeffs[1])+(v/Lf)*(steer_value)*latency;		  
		  
		  Eigen::VectorXd state(6);
		  state<<x_l,y_l,psi_l,v_l,cte_l,epsi_l;
		  
		  auto vars=mpc.Solve(state,coeffs);
		  
		  //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
		  
		  double inc=2.5; // distance increment
		  int num_pts=25;
		  
		  for (int i=0;i<num_pts;i++) {		  
			next_x_vals.push_back(inc*i);
			next_y_vals.push_back(polyeval(coeffs,inc*i));		  
		  }		  
		  
		  //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
		  
		  for (int i=2;i<vars.size();i++) { // refer to vars construction in MPC.cpp.. first 2 terms are steer and pedal
			if(i%2 == 0) {
				mpc_x_vals.push_back(vars[i]);			
			}
			else {
				mpc_y_vals.push_back(vars[i]);			
			}		  
		  }		  		  

          json msgJson; // send to simulator object
		  
          msgJson["steering_angle"] = -vars[0];//negative sign to stay consistent with simulator convention
          msgJson["throttle"] = vars[1];

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
                    
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
