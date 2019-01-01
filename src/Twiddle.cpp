#include "Twiddle.h"
#include <stdio.h>

Twiddle::Twiddle() {}
Twiddle::~Twiddle() {}

void Twiddle::init(double *p_init,double *dp_init,double *best_p_init){
  n = 0;
  max_n = 600;
  total_cte = 0.0;
  error = 0.0;
  best_error = 10000.00;
  tol = 0.001;
  p_iterator = 0;
  total_iterator = 0;
  sub_move = 0;
  first = true;
  second = true;
  p = p_init;
  dp = dp_init;
  best_p = best_p_init;

}

double Twiddle:: twiddle(double cte,PID &pid,uWS::WebSocket<uWS::SERVER> &ws){
  double steer_value;
  json msgJson;

  total_cte = total_cte + pow(cte,2);

  if(n==0){
    pid.Init(p[0],p[1],p[2]);
  }
  //Steering value
  pid.UpdateError(cte);
  steer_value = pid.TotalError();

  n = n+1;
  if (n > max_n) {

    //std::cout << "sump: " << sump << " ";
    if (first == true) {
      std::cout << "Intermediate p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
      p[p_iterator] += dp[p_iterator];
      first = false;
    }else{
      error = total_cte / max_n;

      if (error < best_error && second == true) {
        best_error = error;
        best_p[0] = p[0];
        best_p[1] = p[1];
        best_p[2] = p[2];
        dp[p_iterator] *= 1.1;
        sub_move += 1;
        std::cout << "iteration: " << total_iterator << " ";
        std::cout << "p_iterator: " << p_iterator << " ";
        std::cout << "p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
        std::cout << "error: " << error << " ";
        std::cout << "best_error: " << best_error << " ";
        std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << " " << best_p[1] << " " << best_p[2] << " ";
      } else {
        //std::cout << "else: ";
        if (second == true) {
          std::cout << "Intermediate p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
          p[p_iterator] -= 2 * dp[p_iterator];
          second = false;
        } else{
          std::cout << "iteration: " << total_iterator << " ";
          std::cout << "p_iterator: " << p_iterator << " ";
          std::cout << "p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
          if (error < best_error) {
            best_error = error;
            best_p[0] = p[0];
            best_p[1] = p[1];
            best_p[2] = p[2];
            dp[p_iterator] *= 1.1;
            sub_move += 1;
          } else {
            p[p_iterator] += dp[p_iterator];
            dp[p_iterator] *= 0.9;
            sub_move += 1;
          }
          std::cout << "error: " << error << " ";
          std::cout << "best_error: " << best_error << " ";
          std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << " " << best_p[1] << " " << best_p[2] << " ";
        }
      }

    }

    if (sub_move > 0) {
      p_iterator = p_iterator + 1;
      first = true;
      second = true;
      sub_move = 0;
    }
    if (p_iterator == 3) {
      p_iterator = 0;
    }
    total_cte = 0.0;
    n = 0;
    total_iterator = total_iterator + 1;

    double sumdp = dp[0] + dp[1] + dp[2];
    std::cout << "sumdp: " << sumdp << " ";
    if (sumdp < tol) {
      std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << best_p[1] << best_p[2] << " ";
      ws.close();
    } else {
      reset(ws);
    }
  } else {
    msgJson["steering_angle"] = steer_value;
    msgJson["throttle"] = 0.3;
    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  }
}

void Twiddle::reset(uWS::WebSocket<uWS::SERVER> &ws){
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}