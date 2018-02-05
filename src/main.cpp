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

// Evaluate derivative of a polynomial.
double dpolyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 1; i < coeffs.size(); i++) {
        result += i * coeffs[i] * pow(x, i-1);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
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
    
    // Polynomial order for fitting
    // A second order polynomial is sufficient for the MPC predictons.
    int order_N = 2;
    
    // time duration of prediction step
    int duration = 0;
    
    h.onMessage([&mpc, &order_N, &duration](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
                    // Start a timer to observe how long it takes to execute the MPC
                    //
                    clock_t start = clock();
                    // j[1] is the data JSON object
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];
                    double delta = j[1]["steering_angle"];
                    delta =-delta;  // Switch sign because steering angle is left handed but model is righ handed
                    
                    // Assuming a latency of 100ms update the
                    // the cars state for this latency.
                    const int actuator_latency = 100;   // actuator latency in ms
                    double latency = actuator_latency + duration; // latency of prediction steps
                    double vmps = v*0.44704*.001;       // convert velocity from miles per hour to meters per msec
                    px += vmps*cos(psi)*latency;        // x position after latency period
                    py += vmps*sin(psi)*latency;        // y position after latency period
                    psi += vmps*delta/2.67*latency;     // heading after latency period (Lf = 2.67)
                    
                    // Transform ptsx from map coordinates to car coordinates
                    // We are applying the transform to the state of the car
                    // after the latency period.
                    double cos_psi = cos(psi);   // Note: cos(-psi) = cos(psi)
                    double sin_psi =-sin(psi);   // Note: sin(-psi) =-sin(psi)
                    for (int i = 0; i < ptsx.size(); i++ )
                    {
                        // Transform waypoint i from global to car coordinates.
                        double x = ptsx[i] - px;
                        double y = ptsy[i] - py;
                        
                        double xt = x*cos_psi - y*sin_psi;
                        double yt = x*sin_psi + y*cos_psi;
                        
                        ptsx[i] = xt;
                        ptsy[i] = yt;
                    }
                    
                    // Cast ptsx and ptsy from vector to Eigen::VectorXd
                    Eigen::VectorXd xvals = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
                    Eigen::VectorXd yvals = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());
                    
                    // Now fit data to polynominal of order order_N
                    auto coeffs = polyfit(xvals, yvals, order_N);
                    cout << " Coef[2] " << coeffs[2] << endl;
                    
                    // The cross track error is calculated by evaluating at polynomial f(x) for x=0.
                    // Note that we have tranformed the corrdiantes into car coordinates thus the
                    // current (x,y) position of the car is at (0,0).
                    double cte = polyeval(coeffs, 0) - 0;
                    
                    // Now calcuate the heading error (desired heading psi - actual heading).
                    // The desired heading is arc tan of the tangent to the fit curve evaluated at the
                    // current car position (x,y).  Like cte, becuase the coordinates have been transformed
                    // the current car position is at (0,0) and the car heading psi is also 0.
                    // Thus, the error in the heading epsi = psi - atan(f'(x)) is epsi = 0 - atan(f'(0)).
                    // First step is to create an eigen vector of order order_N - 1 and
                    // store the derivate values.
                    double epsi = 0 - atan(dpolyeval(coeffs, 0));
                    
                    cout << "CTE " << cte << " EPSI " << epsi << endl;
                    
                    // Now store the current state of the car
                    // Note becuase of the coordinate transform the current state is
                    // x = 0, y = 0, and psi = 0.  It is assumed that the change in velocity
                    // during the latency period is negligable.  The error states cte and epsi are
                    // those that were just calcuated following the latency period.
                    Eigen::VectorXd state(6);
                    state << 0, 0, 0, v, cte, epsi;
                    
                    // Solve the Model Predictive Control given the current state and the
                    // desired path as defined by coeffs.  The solver returns the follwoing:
                    // results[0] = steer_value
                    // results[1] = throttle
                    // results[2..2N+1]    = predicted (x,y) path (in car coordinates)
                    auto results = mpc.Solve(state, coeffs);
                    
                    // Extract the new steering and throttle values
                    // Note that the steering value needs to be converted from radians
                    // to the actuator value by the ratio actuator = 1 is equivalent to 25 degree
                    // steering input.  Also the steering value needs to be negated because
                    // the state equations are
                    double steer_value =-results[0]/deg2rad(25.);
                    double throttle_value = results[1];
                    
                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;
                    
                    cout << "STEER " << steer_value << " THROTTLE " << throttle_value << endl;
                    
                    //Display the MPC predicted trajectory
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;
                    for(int i=2; i<results.size(); i+=2) {
                        mpc_x_vals.push_back(results[i]);
                        mpc_y_vals.push_back(results[i+1]);
                    }
                    
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line
                    
                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;
                    
                    // Display the waypoints/reference line
                    // Note the first waypoint (i=0) is neglected becaue of the
                    // latency period has generally caused the car to move past this
                    // waypoint.
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
                    for(int i=1; i<ptsx.size(); i++){
                        next_x_vals.push_back(ptsx[i]);
                        next_y_vals.push_back(ptsy[i]);
                    }
                    
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line
                    
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;
                    
                    
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";

                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    duration = int(1000.*( clock() - start ) / (double) CLOCKS_PER_SEC);
                    cout<<"elapsed time: "<< duration << " msec" << endl;
                    
                    // Adjust sleep to compenstate for the execution time of the MPC
                    // so that the update nearly matches the desired latency.
                    this_thread::sleep_for(chrono::milliseconds(actuator_latency));
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
