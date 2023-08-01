#ifndef sub_systems_hpp
#define sub_systems_hpp
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/misc.hpp"

// ---------------- Motors ---------------- //
extern pros::Controller Master;
extern pros::Motor LeftFront;
extern pros::Motor LeftMiddle;
extern pros::Motor LeftBack;
extern pros::Motor RightFront;
extern pros::Motor RightMiddle;
extern pros::Motor RightBack;
extern pros::Motor Shooter1;
extern pros::Motor Shooter2;
extern pros::MotorGroup Left;
extern pros::MotorGroup Right;
extern pros::MotorGroup Shooter;

// ---------------- Sensors ---------------- //
extern pros::ADIEncoder TW_left;
extern pros::ADIEncoder TW_right;
extern pros::ADIEncoder TW_side;
extern pros::ADIDigitalOut Indexer;
extern pros::ADIDigitalIn Limit;
extern pros::Imu Gyro;

#endif /*sub_systems_hpp*/