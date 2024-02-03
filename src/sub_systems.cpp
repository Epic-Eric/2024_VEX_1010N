#include "main.h"
#include "pros/gps.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "sub_systems.hpp"

// ---------------- Motors ---------------- //
pros::Controller Controller(pros::E_CONTROLLER_MASTER);
pros::Motor LeftFront(2, MOTOR_GEAR_BLUE, true);
pros::Motor LeftMiddle(5, MOTOR_GEAR_BLUE);
pros::Motor LeftBack(6, MOTOR_GEAR_BLUE, true);
pros::Motor RightFront(20, MOTOR_GEAR_BLUE);
pros::Motor RightMiddle(11, MOTOR_GEAR_BLUE,true);
pros::Motor RightBack(3, MOTOR_GEAR_BLUE);
pros::Motor Intake(15, MOTOR_GEAR_BLUE, true);
pros::Motor Cata(14, MOTOR_GEAR_RED, true);
pros::MotorGroup Left ({LeftFront,LeftMiddle,LeftBack});
pros::MotorGroup Right ({RightFront,RightMiddle,RightBack});

// ---------------- Sensors ---------------- //
// pros::ADIEncoder TW_front ({7,'G', 'H'}, true);
// pros::ADIEncoder TW_side ('G', 'H');
pros::ADIDigitalOut Left_wing('E');
pros::ADIDigitalOut Right_wing('E');
pros::ADIDigitalOut Left_intake('H');
pros::ADIDigitalOut Right_intake('H');
pros::ADIDigitalOut PTO('A');
pros::ADIDigitalOut Right_blocker('D');
pros::ADIDigitalOut Left_blocker('B');
pros::Imu Gyro (19);
pros::Rotation Cata_Rotation (16);
pros::Rotation TW_front (4);
pros::Gps GPS(13, 0, 0.1651);

// pros::ADIEncoder turnTable ('G', 'H');