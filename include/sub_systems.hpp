#ifndef sub_systems_hpp
#define sub_systems_hpp
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include "pros/rotation.hpp"
#include "okapi/api.hpp"

// ---------------- Motors ---------------- //
extern pros::Controller Controller;
extern pros::Motor LeftFront;
extern pros::Motor LeftMiddle;
extern pros::Motor LeftBack;
extern pros::Motor RightFront;
extern pros::Motor RightMiddle;
extern pros::Motor RightBack;
extern pros::Motor Intake;
extern pros::Motor Cata;
extern pros::MotorGroup Left;
extern pros::MotorGroup Right;

// ---------------- Sensors ---------------- //
extern pros::ADIEncoder TW_front;
extern pros::ADIEncoder TW_side;
extern pros::ADIDigitalOut Left_wing;
extern pros::ADIDigitalOut Right_wing;
extern pros::ADIDigitalOut Left_intake;
extern pros::ADIDigitalOut Right_intake;
extern pros::Imu Gyro;
extern pros::Rotation Cata_Rotation;

#endif /*sub_systems_hpp*/