#include "main.h"

// ---------------- Path generation ---------------- //
Path_Gen::Path_Gen(std::vector<init_node> &init_points){
    var_init(init_points, spacing, smooth_b, smooth_a, tolerance, path_max_vel, curv_k, path_max_accel, look_ahead_dis);
}
Path_Gen::Path_Gen(std::vector<init_node> &init_points, double path_max_vel2, double path_max_acce3){
    var_init(init_points, spacing, smooth_b, smooth_a, tolerance, path_max_vel2, curv_k, path_max_acce3, look_ahead_dis);
}
Path_Gen::Path_Gen(std::vector<init_node> &init_points, double path_max_vel2, double path_max_acce3, double look_ahead_dis4){
    var_init(init_points, spacing, smooth_b, smooth_a, tolerance, path_max_vel2, curv_k, path_max_acce3, look_ahead_dis4);
}
Path_Gen::Path_Gen(std::vector<init_node> &init_points, double look_ahead_dis2){
    var_init(init_points, spacing, smooth_b, smooth_a, tolerance, path_max_vel, curv_k, path_max_accel, look_ahead_dis2);
}

void Path_Gen::var_init(std::vector<init_node> &init_points, int spacing, double smooth_b, double smooth_a, double tolerance, double path_max_vel, double curv_k, double path_max_accel, double look_ahead_dis){
    this->init_points = &init_points;
    this->spacing = spacing;
    this->smooth_b = smooth_b;
    this->smooth_a = smooth_a;
    this->tolerance = tolerance;
    this->path_max_vel = path_max_vel;
    this->curv_k = curv_k;
    this->path_max_accel = path_max_accel;
    this->look_ahead_dis = look_ahead_dis;
    path_init();
}

/*
 @desc: inject points at a given interval along the paths
 @param 1: spacing determines how wide apart the injected points are (inch)
 @param 2: init_points are user inputted way-points
*/
std::vector<init_node> Path_Gen::point_injector (int spacing, std::vector<init_node> init_points){
    std::vector<init_node> filled_points;
    for(int i=0; i<init_points.size()-1; i++){
        
        //variable storage
        double x1 = init_points[i].x;
        double x2 = init_points[i+1].x;
        double y1 = init_points[i].y;
        double y2 = init_points[i+1].y;
        
        //calculate x and y difference
        double dx = x2 - x1;
        double dy = y2 - y1;
        
        //number of points that fit on a line with given spacing
        double mag = magnitude(dx, dy);
        int num_points = ceil(mag / spacing);
        
        //normalize vector
        dx = dx / mag * spacing;
        dy = dy / mag * spacing;
        
        for(int j=0; j<num_points; j++){
            //fill points
            double nx = x1 + dx*j;
            double ny = y1 + dy*j;
            filled_points.push_back({nx,ny});
        }
    }
    //get the last point in path
    filled_points.push_back({init_points[init_points.size()-1].x, init_points[init_points.size()-1].y});
    return filled_points;
}

/*
 @desc: smooths out the points in the path
 @param 1: a vector that stores {x,y} coords of already space-filled points
 @param 2: a is weight data, 1-b
 @param 3: b is weight smooth, larger is more smooth, 0.75 to 0.98 work well
 @param 4: tolerance, set to 0.001
*/
std::vector<node> Path_Gen::path_smoother(std::vector<init_node> init_points, double a, double b, double tolerance){
    //copy vector
    std::vector<node> new_path;
    node temp;
    for(auto x:init_points){
        temp.x = x.x;
        temp.y = x.y;
        new_path.push_back(temp);
    }
    
    double change = tolerance;
    while(change >= tolerance){
        change = 0.0;
        for(int i=1; i<init_points.size()-1; i++){
            //x-coord change
            double auxx = new_path[i].x;
            new_path[i].x += a * (init_points[i].x - new_path[i].x) + b * (new_path[i-1].x + new_path[i+1].x - (2.0 * new_path[i].x));
            change += abs(auxx - new_path[i].x);
            
            //y-coord change
            double auxy = new_path[i].y;
            new_path[i].y += a * (init_points[i].y - new_path[i].y) + b * (new_path[i-1].y + new_path[i+1].y - (2.0 * new_path[i].y));
            change += abs(auxy - new_path[i].y);
        }
    }
    return new_path;
}

/*
 @desc: initialize each point's index along the path (0~size-1)
 @param 1: a pointer to a vector that stores {x,y} coords of injected points on a smooth path
*/
void Path_Gen::index_init (std::vector<node> *path){
    for(int i=0; i<path->size(); i++){
        path->at(i).index = i;
    }
}

/*
 @desc: calculate each points' distance from the start (prefix sum)
 @param 1: a pointer to a vector that stores {x,y} coords of injected points on a smooth path
*/
void Path_Gen::path_dis_calc (std::vector<node> *path){
    path->at(0).dis = 0;
    for(int i=1; i<path->size(); i++){
        double dx = path->at(i).x - path->at(i-1).x;
        double dy = path->at(i).y - path->at(i-1).y;
        
        //prefix sum, keeping a running sum of path distance
        path->at(i).dis = path->at(i-1).dis + magnitude(dx, dy);
    }
}

/*
 @desc: calculate each path point's curvature by finding the radius that a circle could intersect all 3 points, i-1, i, and i+1. There is always one and only one way to circumscribe a triangle, meaning that there's always one given circle for each set of 3 points, given that they make a triangle (not on a stright line)
 @param 1: a pointer to a vector that stores {x,y} coords of injected points on a smooth path with i(x1,y1), i-1(x2,y2), and i+1(x3,y3)
 @note: edge cases if x1 = x2, so we add 0.001 to x1 so we don't divide by 0. If answer is NaN, the radius is ∞, curvature is 0, straight line
*/
void Path_Gen::path_curv_calc (std::vector<node> *path){
    //start and end points have 0 curvature
    path->at(0).curvature = 0;
    path->at(path->size()-1).curvature = 0;
    
    for(int i=1; i<path->size()-1; i++){
        //variable initialization
        double x1 = path->at(i).x;
        double y1 = path->at(i).y;
        double x2 = path->at(i-1).x;
        double y2 = path->at(i-1).y;
        double x3 = path->at(i+1).x;
        double y3 = path->at(i+1).y;
        
        //edge case
        x1+=0.001;
        
        //calculation
        double k1 = 0.5*(x1*x1+y1*y1-x2*x2-y2*y2)/(x1-x2);
        double k2 = (y1-y2)/(x1-x2);
        double b = 0.5*(x2*x2-2*x2*k1+y2*y2-x3*x3+2*x3*k1-y3*y3)/(x3*k2-y3+y2-x2*k2);
        double a = k1-k2*b;
        double r = magnitude((x1-a), (y1-b));
        double curv = 1/r;
        
        path->at(i).curvature = curv;
    }
}

/*
 @desc: sets the drivetrain's maximum velocity, according to path max velocity (user defined) and curvature
 @param 1: the user-provided maximum velocity on the path (inch/s)
 @param 2: k value, used in k/curvature, the bigger the k, the faster robot goes around sharp corners, 1~5
 @param 3: path points vector pointer
*/
void Path_Gen::velo_curv_constr (double path_max_vel, double k, std::vector<node> *path){
    for(int i=0; i<path->size(); i++){
        path->at(i).velocity = std::min(path_max_vel, k/path->at(i).curvature);
    }
    
    //stop at the last point
    path->at(path->size()-1).velocity = 0;
}

/*
 @desc: sets the drivetrain's maximum velocity, according to previous velocity from curvature and maximum acceleration, using kinematic equation of vf^2 = vi^2 + 2 * a * d. vf = final velocity, vi = initial velocity, a = max acceleration / deceleration, d = dis between point. For example, robot can't stop from 80inch/s to 0inch/s in 4 inch space, so need to decrease 80inch/s so it can. Loop from end to start
 @param 1: the user-provided maximum acceleration on the path (small value of a for slower acceleration, also depends on the robot, inch / s^2)
 @param 2: path points vector pointer
*/
void Path_Gen::velo_accel_constr (double path_max_accel, std::vector<node> *path){
    for(int i=int(path->size()-2); i>=0; i--){
        //variable initialization
        double x1 = path->at(i).x; //init
        double y1 = path->at(i).y;
        double x2 = path->at(i+1).x; //final
        double y2 = path->at(i+1).y;
        
        //calculation
        double dis = magnitude((x2-x1), (y2-y1));
        double temp = sqrt(path->at(i+1).velocity*path->at(i+1).velocity+2*path_max_accel*dis);
        path->at(i).velocity = std::min(path->at(i).velocity, temp);
    }
}

void Path_Gen::path_init(){
    filled_points = point_injector(spacing, *init_points);
    path = path_smoother(filled_points, smooth_a, smooth_b, tolerance);
    index_init(&path);
    path_dis_calc(&path);
    path_curv_calc(&path);
    velo_curv_constr(path_max_vel, curv_k, &path);
    velo_accel_constr(path_max_accel, &path);
}

void Path_Gen::debug(){
    //initial way-points
    std::cout<<"Initial way-points"<<std::endl;
    for(auto x: *init_points){
        printf("(%lf,%lf),", x.x,x.y);
    }
    printf("\n\n");
    
    //injected points
    std::cout<<"Injected points"<<std::endl;
    for(auto x: filled_points){
        printf("(%lf,%lf),", x.x,x.y);
    }
    printf("\n\n");
    
    //smooth points
    std::cout<<"Smooth points"<<std::endl;
    for(auto x: path){
        printf("(%lf,%lf),", x.x,x.y);
    }
    printf("\n\n");
    
    //velocity of each point
    std::cout<<"Point velocity"<<std::endl;
    for(auto x: path){
        printf("(%d,%lf),", x.index,x.velocity);
    }
    printf("\n\n");
}