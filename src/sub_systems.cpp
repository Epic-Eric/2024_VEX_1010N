#include "main.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "sub_systems.hpp"

// ---------------- Motors ---------------- //
pros::Controller Controller(pros::E_CONTROLLER_MASTER);
pros::Motor LeftFront(2, MOTOR_GEAR_BLUE);
pros::Motor LeftMiddle(9, MOTOR_GEAR_BLUE);
pros::Motor LeftBack(11, MOTOR_GEAR_BLUE);
pros::Motor RightFront(20, MOTOR_GEAR_BLUE, true);
pros::Motor RightMiddle(19, MOTOR_GEAR_BLUE, true);
pros::Motor RightBack(17, MOTOR_GEAR_BLUE, true);
pros::Motor Intake(16, MOTOR_GEAR_BLUE);
pros::Motor Cata(10, MOTOR_GEAR_RED, true);
pros::MotorGroup Left ({LeftFront,LeftMiddle,LeftBack});
pros::MotorGroup Right ({RightFront,RightMiddle,RightBack});

// ---------------- Sensors ---------------- //
pros::ADIEncoder TW_front ({7,'G', 'H'});
pros::ADIEncoder TW_side ({7,'E', 'F'});;
pros::ADIDigitalOut Left_wing('A');
pros::ADIDigitalOut Right_wing('B');
pros::ADIDigitalOut Left_intake('C');
pros::ADIDigitalOut Right_intake('D');
pros::Imu Gyro (8);
pros::Rotation Cata_Rotation (1);