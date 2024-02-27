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
// extern pros::ADIEncoder TW_front;
// extern pros::ADIEncoder TW_side;
extern pros::ADIDigitalOut Left_wing;
extern pros::ADIDigitalOut Right_wing;
extern pros::ADIDigitalOut Left_intake;
extern pros::ADIDigitalOut Right_intake;
extern pros::ADIDigitalOut Right_back_wing;
extern pros::ADIDigitalOut Left_back_wing;
extern pros::Imu Gyro;
extern pros::Rotation Cata_Rotation;
extern pros::Rotation TW_front;
extern pros::Gps GPS;

// extern pros::ADIEncoder turnTable;

#endif /*sub_systems_hpp*/

// //process
//         if(!manual_control){ // during normal match play
//             if(rapid_fire){
//                 if(cata_speed>speed_lim){ //too fast! need to wait till cata is back so no caught in mid-shot
//                     cata_released = true;
//                     Cata.brake();
//                 }
//                 else if(cata_released && !(cata_ang>top_ang-range)){ //if it's shot but hasn't returned ?! (is it stuck?) motor doesn't spin
//                     Cata.brake();
//                 }
//                 else {
//                     cata_released = false;
//                     Cata.move_velocity(100);
//                 }
//             }
//             else if(shot && (cata_ang<range&&cata_ang>0 || cata_ang<360&&cata_ang>360-range)){ //if cata ang in range of dead stop needs to shoot
//                 Cata.move_velocity(100);
//                 cata_released = true;
//             }
//             else if(cata_ang<range&&cata_ang>0 || cata_ang<360&&cata_ang>360-range){ //if cata ang in range of dead stop needs to stop
//                 Cata.brake();
//             }
//             else if(cata_released && !(cata_ang>top_ang-range)){ //if it's shot but hasn't returned ?! (is it stuck?) motor doesn't spin
//                 Cata.brake();
//             }
//             else{ //if between raised and dead limit
//                 cata_released = false;
//                 Cata.move_velocity(100); //always move down
//             }
//         }
//         else{ // when wants to climb, needs manual control
//             if(shot){
//                 Cata.move_velocity(100);
//             }
//             else Cata.brake();
//         }
//         //std::cout<<cata_speed<<std::endl;