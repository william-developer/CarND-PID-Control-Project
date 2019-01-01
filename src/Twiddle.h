#ifndef PID_TWIDDLE_H
#define PID_TWIDDLE_H
#include "PID.h"
#include "json.hpp"
#include <uWS/uWS.h>
#include <math.h>
#include <iostream>

using json = nlohmann::json;


class Twiddle {
public:
    Twiddle();
    virtual ~Twiddle();

    double *p =new double[3];
    double *dp =new double[3];
    int n;
    int max_n;
    double total_cte;
    double error;
    double best_error;
    double tol;
    int p_iterator;
    int total_iterator;
    int sub_move;
    bool first;
    bool second;
    double *best_p = new double[3];

    void init(double *p,double *dp,double *best_p);
    double twiddle(double cte,PID &pid,uWS::WebSocket<uWS::SERVER> &ws);
    void reset(uWS::WebSocket<uWS::SERVER> &ws);
};


#endif //PID_TWIDDLE_H
