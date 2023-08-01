#include "main.h"

// ---------------- Functions---------------- //
double magnitude (double x, double y){
    return (sqrt(x*x+y*y));
}

int signum (double num){
    if(num>0) return 1;
    else if(num<0) return -1;
    return 0;
}

/*
 @desc: turns inch/s to rpm which inputs to the drivetrain
 @formula: rpm * gear ratio / 60 * wheel_dia * π = velo
 @param 1: wheel diameter
 @param 2: velocity in inch/s
 @param 3: motor gear : driven gear
*/
double inch_per_sec_to_rpm (double wheel_dia, double velo, double gear_ratio){
    return (velo/M_PI/wheel_dia*60/gear_ratio);
}

double rpm_to_inch_per_sec (double wheel_dia, double rpm, double gear_ratio){
    return (rpm*gear_ratio/60*wheel_dia*M_PI);
}

double average (std::vector<double>){
    
}