#ifndef robot_hpp
#define robot_hpp

#include <stdio.h>
#include <vector>
#include <math.h>
#include <iostream>

// ---------------- Robot ---------------- //
class Robo{
public:
    //variables
    double track_width, wheel_dia, gear_ratio; //inch, inch, motor:driven
    double x,y,theta; //theta is in radian 0~2π

    //constructor
    Robo(double track_width, double wheel_dia, double gear_ratio);
    
    //functions
    void robot_update (double x, double y, double theta);
    void debug();
};

#endif /* robot_hpp */