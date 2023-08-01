#ifndef path_gen_hpp
#define path_gen_hpp

#include <stdio.h>
#include <vector>
#include <math.h>
#include <iostream>

// ---------------- Path generation ---------------- //
struct init_node{
    double x,y;
};
struct node{
    int index;
    double x,y,dis,curvature,velocity;
};
class Path_Gen{
public:
    //variables
    int spacing = 4; //inch
    double smooth_b = 0.80;
    double smooth_a = 1.0 - smooth_b;
    double tolerance = 0.001;
    double path_max_vel = 76.6; //inch / s
    double curv_k = 4; //1~5
    double path_max_accel = 8.55; //inch / s^2
    double look_ahead_dis = 15; //12~25 inch
    std::vector<init_node> *init_points;
    std::vector<init_node> filled_points;
    
    //functions
    std::vector<init_node> point_injector (const int spacing, std::vector<init_node> init_points);
    std::vector<node> path_smoother(std::vector<init_node> init_points, double a, double b, double tolerance);
    void index_init (std::vector<node> *path);
    void path_dis_calc (std::vector<node> *path);
    void path_curv_calc (std::vector<node> *path);
    void velo_curv_constr (double path_max_vel, double k, std::vector<node> *path);
    void velo_accel_constr (double path_max_accel, std::vector<node> *path);
    
    //constructors
    Path_Gen(std::vector<init_node> &init_points);
    Path_Gen(std::vector<init_node> &init_points, double path_max_vel, double path_max_accel);
    Path_Gen(std::vector<init_node> &init_points, double path_max_vel, double path_max_accel, double look_ahead_dis);
    Path_Gen(std::vector<init_node> &init_points, double look_ahead_dis);
    
    //variables
    std::vector<node> path;
    
    //functions
    void var_init(std::vector<init_node> &init_points, const int spacing, const double smooth_b, const double smooth_a, const double tolerance, double path_max_vel, double curv_k, double path_max_accel, double look_ahead_dis);
    void path_init();
    void debug();
};


#endif /* path_gen_hpp */