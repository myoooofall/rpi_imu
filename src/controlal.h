#ifndef CONTROLAL_H
#define CONTROLAL_H

#include <thread>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <iomanip>
#include <cmath>
#include <stdlib.h>
#include <chrono>
#include <unistd.h>

using namespace std;

class controlal{
public:

    enum Mode {
        RealTime,
        Table
    };
    
    enum PlanType {
        X,
        Y,
        ROTATE
    };
    //debug module
    bool debug_flag = true;
    double CaseX = 0.0;//debug
    double CaseY = 0.0;

    //control rate
    double dt = 0.002; //s

    //for Table mode
    vector<float> accTable;
    vector<float> velTable;
    vector<float> posTable;
    
    //para
    double a_max = 20000;
    double v_max = 3000;
    double d_max = 20000; //mm/s
    
    //Input
    double x1;
    double y1;//目标位置(相对当前位置)

    double v0_x;
    double v0_y;//当前速度

    double v1_x;
    double v1_y;//目标速度

    //time
    double traj_time_x = 0;
    double traj_time_acc_x = 0;
    double traj_time_dec_x = 0;
    double traj_time_flat_x = 0;
    double traj_time_y = 0;
    double traj_time_acc_y = 0;
    double traj_time_dec_y = 0;
    double traj_time_flat_y = 0;

    void control_init();

private:

    
    double a[2] = {0}; //ax,ay calculate result for realtime
    double v[2] = {0}; //vx,vy calculate result for realtime
    double delta_pos[2] = {0}; //x,y calculate result for realtime
    double compute_motion_1d(double x1, double v0, double v1,
                       const double a_max, const double d_max, const double v_max,
                       double &traj_time, double &traj_time_acc, double &traj_time_dec, double &traj_time_flat,
					   Mode mode,double dt,PlanType PT);
    double get_t_max(double x1, double v0, double v1, double a_max, double d_max, double v_max);
    void compute_motion_2d(double &x1, double &v0_x, double v1_x, 
                       double &y1, double &v0_y, double v1_y,
                       const double a_max, const double d_max, const double v_max,
                       double &traj_time_x, double &traj_time_acc_x, double &traj_time_dec_x, double &traj_time_flat_x,
                       double &traj_time_y, double &traj_time_acc_y, double &traj_time_dec_y, double &traj_time_flat_y,
					   Mode mode,double dt);
    void local_planner_thread_func();
    auto now();

};
#endif