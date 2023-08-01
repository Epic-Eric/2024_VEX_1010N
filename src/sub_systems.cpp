#include "main.h"
#include "pros/motors.h"
#include "sub_systems.hpp"

// ---------------- Motors ---------------- //
pros::Controller Master(pros::E_CONTROLLER_MASTER);
pros::Motor LeftFront(1, MOTOR_GEAR_BLUE);
pros::Motor LeftMiddle(2, MOTOR_GEAR_BLUE);
pros::Motor LeftBack(3, MOTOR_GEAR_BLUE);
pros::Motor RightFront(5, MOTOR_GEAR_BLUE, true);
pros::Motor RightMiddle(7, MOTOR_GEAR_BLUE, true);
pros::Motor RightBack(8, MOTOR_GEAR_BLUE, true);
pros::Motor Shooter1(9, MOTOR_GEAR_BLUE);
pros::Motor Shooter2(10, MOTOR_GEAR_BLUE);
pros::MotorGroup Left ({LeftFront,LeftMiddle,LeftBack});
pros::MotorGroup Right ({RightFront,RightMiddle,RightBack});
pros::MotorGroup Shooter ({Shooter1, Shooter2});

// ---------------- Sensors ---------------- //
pros::ADIEncoder TW_left ('A', 'B');
pros::ADIEncoder TW_right ('C', 'D');
pros::ADIEncoder TW_side ('E', 'F');;
pros::ADIDigitalOut Indexer ('G');
pros::ADIDigitalIn Limit ('H');
pros::Imu Gyro (9);