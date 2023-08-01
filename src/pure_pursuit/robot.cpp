#include "main.h"

// ---------------- Robot ---------------- //
Robo::Robo(double track_width, double wheel_dia, double gear_ratio){
    this->track_width = track_width;
    this->wheel_dia = wheel_dia;
    this->gear_ratio = gear_ratio;
}

void Robo::robot_update(double x, double y, double theta){
    this->x = x;
    this->y = y;
    this->theta = theta;
}

void Robo::debug(){
    //print x, y, and theta
    printf("X: %lf, Y: %lf, Theta: %lf\n\n", x, y, theta);
}