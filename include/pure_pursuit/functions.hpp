#ifndef functions_hpp
#define functions_hpp

#include <stdio.h>
#include <vector>
#include <math.h>
#include <iostream>

// ---------------- Functions ---------------- //
double magnitude (double x, double y);
int signum (double num);
double inch_per_sec_to_rpm (double wheel_dia, double velo, double gear_ratio);
double rpm_to_inch_per_sec (double wheel_dia, double rpm, double gear_ratio);
double average (std::vector<double>);

#endif /* functions_hpp */