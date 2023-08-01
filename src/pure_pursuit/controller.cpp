#include "main.h"

// ---------------- Controller ---------------- //
Control::Control(double Kv, double Ka, double Kp){
    this->Kv = Kv;
    this->Ka = Ka;
    this->Kp = Kp;
}

/*
 @desc: determine which waypoint is closest to robot, providing look-up velocity
 @param 1: a pointer to the robot object
 @param 2: a pointer to the path that the robot follows
*/
void Control::find_closest (Control *controller, Robo *robot, std::vector<node> *path){
    double min_dis = 999;
    for(int i=controller->closest_point; i<path->size(); i++){
        //varialble initialization
        double x1 = robot->x;
        double y1 = robot->y;
        double x2 = path->at(i).x;
        double y2 = path->at(i).y;
        
        //find the closest
        double dis = magnitude((x2-x1), (y2-y1));
        if(dis<min_dis){
            controller->closest_point = i;
        }
        min_dis = std::min(min_dis, dis);
    }
}

/*
 @desc: we find the lookahead point by finding the intersection point of the circle of radius lookahead distance centered at robot, and each path segments. It calculates 2 possible intersections (t values) between the circle and a line segment. t is between 0~1 representing how far the point is from the start of the segment. t<0 and t>1 means circle doesn't intersect the segment.
 @param 1&2: e is the starting point of line segment
 @param 3&4: l is the end point of line segment
 @param 5&6: c is the robot center
 @param 7  : r is the radius of circle of lookahead distance centered at robot
*/
double Control::line_circle_intersect (double ex, double ey, double lx, double ly, double cx, double cy, double r){
    //variable initialization
    double dx = lx - ex;
    double dy = ly - ey;
    double fx = ex - cx;
    double fy = ey - cy;
    
    //calculation
    double a = dx*dx + dy*dy;
    double b = 2 * (fx*dx + fy*dy);
    double c = fx*fx + fy*fy - r*r;
    double discriminant = b*b - 4*a*c;
    
    //check
    if(discriminant < 0){
        return 999;
    }
    else{
        discriminant = sqrt(discriminant);
        double t1 = (-b - discriminant) / (2*a);
        double t2 = (-b + discriminant) / (2*a);
        
        if(t1>=0 && t1<=1){
            return t1; //in the case that a robot could have both valid t1&t2, meaning have 2 intersections with a single segment, return both t1 and t2 to prevent getting stuck (only t1 ever gets returned). In this case the r is tested to best be 12~25, significantly higher than the 4-inch spacing
        }
        if(t2>=0 && t2<=1){
            return t2;
        }
    }
    return 999;
}

/*
 @desc: we start at the closest waypoint to avoid unnecessary calculation. A fractional index is given to the lookahead point, which is just the start + t proportion. The 1st fractional index that's greater than the last is chosen to follow, ensures that the robot doesn't go back.
 @param 1: a pointer to the robot object
 @param 2: a pointer to the path that the robot follows
 @param 3: the look ahead distance, circle around the robot
*/
void Control::find_lookahead_point (Control *controller, Robo *robot, std::vector<node> *path, double look_ahead){
    //calculation
    int start = std::floor(controller->look_ahead_point_fractional_index); // 2.5 would begin search at 2
    for(int i=start; i<path->size()-1; i++){
        //variable initialization
        double ex = path->at(i).x;
        double ey = path->at(i).y;
        double lx = path->at(i+1).x;
        double ly = path->at(i+1).y;
        double cx = robot->x;
        double cy = robot->y;
        double r = look_ahead;
        
        //lookup
        double t = line_circle_intersect(ex, ey, lx, ly, cx, cy, r);
        if(t==999) continue; //no intersection
        else{
            double fractional_index = path->at(i).index + t;
            if(fractional_index>controller->look_ahead_point_fractional_index){
                controller->look_ahead_point_fractional_index = fractional_index;
                double dx = lx - ex;
                double dy = ly - ey;
                double px = ex + t * dx;
                double py = ey + t * dy;
                controller->look_ahead_point_x = px;
                controller->look_ahead_point_y = py;
                return;
            }
        }
    }
    //if in the end no found, use the previously stored lookahead point in robot object
}

/*
 @desc: find the curvature from the robot (rx,ry) to look_ahead_point (lx,ly) in order to drive in a smooth arc, with consideration of which side the curvature is. + for right turn, - for left turn
 @param 1: a pointer to the robot object
 @param 2: the look ahead distance
*/
void Control::find_curvature (Control *controller, Robo *robot, double look_ahead){
    //variable initialization
    double rx = robot->x;
    double ry = robot->y;
    double rt = robot->theta; //in radians
    double lx = controller->look_ahead_point_x;
    double ly = controller->look_ahead_point_y;
    
    //calculation
    double a = -tan(rt);
    double b = 1;
    double c = tan(rt) * rx - ry;
    double x = abs(a*lx+b*ly+c) / sqrt(a*a+b*b);
    double curvature = 2*x/(look_ahead*look_ahead);
    
    //determine side
    int side = signum(sin(rt)*(lx-rx)-cos(rt)*(ly-ry));
    double signed_curv = side * curvature;
    controller->curvature = signed_curv;
}

/*
 @desc: determine the speeds for the left and right side of the drivetrain
 @param 1: a pointer to the robot object
 @param 2: the trackwidth of the robot, a bit bigger than wheel center to wheel center, more like wheel edge to wheel edge due to turning scrub
 @param 3: pointer to the path that the robot is following
*/
void Control::find_wheel_speeds (Control *controller, Robo *robot, std::vector<node> *path){
    double v = path->at(controller->closest_point).velocity;
    double c = controller->curvature;
    double t = robot->track_width;
    controller->left_speed = v * (2+c*t)/2;
    controller->right_speed = v * (2-c*t)/2;
}

/*
 @desc: FF and FB controls left side drivetrain speed
 @param 1: a pointer to the robot object
 @param 2: a pointer to the Kv, Ka, Kp control values
*/
void Control::left_control (Control *controller){
    double Kv = controller->Kv;
    double Ka = controller->Ka;
    double Kp = controller->Kp;
    double feed_forward = Kv * controller->left_speed + Ka * controller->left_accel;
    double feed_backward = Kp * (controller->left_speed - controller->robot_left_speed);
    controller->FF_FB_left = feed_forward + feed_backward;
}

/*
 @desc: FF and FB controls right side drivetrain speed
 @param 1: a pointer to the robot object
 @param 2: a pointer to the Kv, Ka, Kp control values
*/
void Control::right_control (Control *controller){
    double Kv = controller->Kv;
    double Ka = controller->Ka;
    double Kp = controller->Kp;
    double feed_forward = Kv * controller->right_speed + Ka * controller->right_accel;
    double feed_backward = Kp * (controller->right_speed - controller->robot_right_speed);
    controller->FF_FB_right = feed_forward + feed_backward;
}

void Control::run(Control *controller, Robo *robot, std::vector<node> *path, double look_ahead){
    find_closest (controller, robot, path);
    find_lookahead_point (controller, robot, path, look_ahead);
    find_curvature (controller, robot, look_ahead);
    find_wheel_speeds (controller, robot, path);
    left_control(controller);
    right_control(controller);
}

void Control::debug(Robo *robot){
    //robot's closest waypoint index
    std::cout<<"Robot's closest waypoint index"<<std::endl;
    std::cout<<closest_point<<std::endl;
    printf("\n");
    
    //robot's lookahead point
    std::cout<<"Robot's lookahead point"<<std::endl;
    printf("index: %lf, x: %lf, y: %lf\n\n", look_ahead_point_fractional_index, look_ahead_point_x, look_ahead_point_y);
    
    //robot's curvature to that point
    std::cout<<"Robot's curvature to that point"<<std::endl;
    printf("curvature to point: %lf\n\n", curvature);
    
    //left and right speed
    std::cout<<"Left and right speed"<<std::endl;
    printf("left: %lf, right: %lf\n\n", left_speed, right_speed);
    
    //K values
    std::cout<<"K values"<<std::endl;
    printf("Kv: %lf, Ka: %lf, Kp: %lf\n\n", Kv, Ka, Kp);
    
    //FF and FB tuned left and right speed
    std::cout<<"FF and FB Tuned left and right speed"<<std::endl;
    printf("tuned left: %lf, tuned right: %lf\n\n", FF_FB_left, FF_FB_right);
    
    //tuned left and right speed
    std::cout<<"FF and FB Tuned left and right rpm"<<std::endl;
    printf("tuned rpm: %lf, tuned rpm: %lf\n\n", inch_per_sec_to_rpm(robot->wheel_dia, FF_FB_left, robot->gear_ratio), inch_per_sec_to_rpm(robot->wheel_dia, FF_FB_right, robot->gear_ratio));
}