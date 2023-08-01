#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "sub_systems.hpp"
#include <iostream>
// ---------------- Stats ---------------- //
std::vector<init_node> points1{
    {0.0,0.0},{-40,40},{-40,60},{0,60},{40,60},{20,0}
};
double track_width = 15; //outer edge of left and right wheel distance
double wheel_dia = 3.25; //wheel diameter
double gear_ratio = (double)36/48; //motor gear : driven gear
double Kv = 0.1; //velocity tuner (feedforward) 
double Ka = 0.1; //acceleration tuner (feedforward)
double Kp = 0.1; //proportional tuner (feedback) 

// ---------------- Objects ---------------- //
//Robot
Robo robobo = {
    track_width, wheel_dia, gear_ratio
};

//Paths
Path_Gen path1 {
	points1
};

//Controller
Control controller {
    Kv, Ka, Kp
};

// ---------------- Graph ---------------- //
// Create grapher
std::shared_ptr<graphy::AsyncGrapher> grapher(new graphy::AsyncGrapher("Drivetrain target vs. actual velocity"));
// void record(){
//     while(true) {
//     //updates the controller
//     controller.robot_left_speed = average(Left.pros::c::motor_get_actual_velocity());
//     controller.run(&controller, &robobo, &path1.path, path1.look_ahead_dis);

//     //updates grapher
//     grapher->update("Left Desired Vel", controller.FF_FB_left);
//     grapher->update("Actual Vel", ACTUAL_VELOCITY); //3 average RPM velocity of one side

//     pros::delay(10);
// }
// }


// When enters the program
void initialize() {
	pros::lcd::initialize();
    robobo.robot_update(-40, 40, 1.57);

    // Add data types
    grapher->addDataType("Desired Vel", COLOR_ORANGE);
    grapher->addDataType("Actual Vel", COLOR_AQUAMARINE);
    grapher->startTask();
    //record();
}

// After / before auto and driver
void disabled() {

}

// After initialize, before auto when plugged in comp switch
// For auto selector etc.
void competition_initialize() {

}


// Robot autonomous tasks
void autonomous() {
    
}


// Driver control
void opcontrol() {
        
    // ---------------- Drivetrain ---------------- //
    double turnImportance = 0.3; //How much turning slows down the speed of forward, 0 doesn't affect, 1 stops forward
    double turnSensitivity = 0.8; //How sensitive a turn is, 0 doesn't turn, 1 most sensitive
    Left.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    Right.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

    while(true){
        // ---------------- Drivetrain ---------------- //
        double motorTurnVal = Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)/(double)127*100; //convert to percentage
        double motorForwardVal = Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)/(double)127*100; //convert to percentage
        //Volts range: -12 --> 12, converts percentage to volts
        double motorTurnVolts = turnSensitivity*(motorTurnVal * 0.12);
        //Times forward volts by a percentage from how much you turn and how important the turn is to slowing down forward speed
        double motorForwardVolts = motorForwardVal * 0.12 * (1 - (std::abs(motorTurnVolts)/12 * turnImportance)); 
        Left.move_voltage ((motorForwardVolts - motorTurnVolts) * (double) 1000);
        Right.move_voltage ((motorForwardVolts + motorTurnVolts) * (double) 1000);
        std::cout<<LeftFront.get_voltage()<<std::endl;
        pros::delay(2);
  }
}
