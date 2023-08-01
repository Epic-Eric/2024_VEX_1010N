#ifndef controller_hpp
#define controller_hpp

#include <stdio.h>
#include <vector>
#include <math.h>
#include <iostream>
#include "robot.hpp"
#include "path_gen.hpp"

// ---------------- Controller ---------------- //
class Control{
public:
    //variables
    double Kv, Ka, Kp; //velocity, acceleration , proportional
    int closest_point;
    double look_ahead_point_fractional_index, look_ahead_point_x, look_ahead_point_y;
    double curvature, left_speed, right_speed, left_accel, right_accel;
    double robot_left_speed, robot_right_speed, FF_FB_left, FF_FB_right;
    
    //constructor
    Control(double Kv, double Ka, double Kp);
    
    //functions
    void find_closest (Control *controller, Robo *robot, std::vector<node> *path);
    double line_circle_intersect (double ex, double ey, double lx, double ly, double cx, double cy, double r);
    void find_lookahead_point (Control *controller, Robo *robot, std::vector<node> *path, double look_ahead);
    void find_curvature (Control *controller, Robo *robot, double look_ahead);
    void find_wheel_speeds (Control *controller, Robo *robot, std::vector<node> *path);
    void left_control (Control *controller);
    void right_control (Control *controller);
    
    void run (Control *controller, Robo *robot, std::vector<node> *path, double look_ahead);
    void debug(Robo *robot);
};

#endif /* controller_hpp */