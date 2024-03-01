#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/pose.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "sub_systems.hpp"
#include <cmath>
#include <cstddef>
#include <valarray>
#define m_to_inch(meter) meter*39.3701;

// ---------------- User functions ---------------- //
bool we_are_blue = false;
bool left_back_wing, right_back_wing = false; 
double middle_ang = 40, down_ang = 58;
bool skills_gps_deadzone(double x, double y){
    if(x< -58.5 || x>58.5 || y< -58.5 || y>58.5) return true; //a square of deadzone
    return false;
}

bool cata_in_range(double current, double target){
    const double cata_range = 5;
    if(std::abs(current - target) < cata_range) return true; //if angle difference < 5 degrees
    if(std::abs((360-current) - target) < cata_range) return true; //edge case where cata is 359 and target is 0
    return false;
}

void set_cata(double target){
    double cata_ang = Cata_Rotation.get_angle()/100.0;
    if(cata_in_range(cata_ang, target)) Cata.brake();
    else Cata.move_velocity(100);
    // pros::lcd::print(4, "now %lf", cata_ang); // print the cata angle
    // pros::lcd::print(5, "target %lf", target); // print the cata target angle
}

void rage_mode(int dir, double time){
    if(dir==0){ //forward
        Left.move(12000);
        Right.move(12000);
    }
    else{
        Left.move(-12000);
        Right.move(-12000);
    }
    pros::delay(time);
}

void intake_out(){
    Left_intake.set_value(1);
    Right_intake.set_value(1);
}
void intake_in(){
    Left_intake.set_value(0);
    Right_intake.set_value(0);
}
void wings_out(){
    Left_wing.set_value(1);
    Right_wing.set_value(1);
}
void wings_in(){
    Left_wing.set_value(0);
    Right_wing.set_value(0);
}
void left_back_wing_out(){
    Left_back_wing.set_value(1);
}
void left_back_wing_in(){
    Left_back_wing.set_value(0);
}
void right_back_wing_out(){
    Right_back_wing.set_value(1);
}
void right_back_wing_in(){
    Right_back_wing.set_value(0);
}

// ---------------- LemLib ---------------- //
lemlib::Drivetrain_t drivetrain {
    &Left, // left drivetrain motors
    &Right, // right drivetrain motors
    11.25, // track width
    3.25, // wheel diameter
    360, // wheel rpm
	12 //chase power
};
lemlib::TrackingWheel front_tracking_wheel(&TW_front, 2.75, -0.375);

// odometry struct - 1 vertical tracking wheel
lemlib::OdomSensors_t sensors {
    &front_tracking_wheel, // vertical tracking wheel 1
    nullptr, // don't have
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &Gyro // inertial sensor
};

// forward/backward PID
lemlib::ChassisController_t lateralController {
    30, // kP //12, 14, 18, 23, 30
    42, // kD //4.5, 15.5, 42, 72, 126
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    7 // slew rate //5
};
 
// turning PID
lemlib::ChassisController_t angularController {
    5, // kP
    35, // kD 24 26
    1, // smallErrorRange
    100, // smallErrorTimeout
    5, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};

// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

void screen() {
    // loop forever
    while (true) {
        // ---------------- GPS Stuff ---------------- //
        auto package = GPS.get_status();
        double gps_x = m_to_inch(package.x);
        double gps_y = m_to_inch(package.y);
        double gps_theta = fmod((package.yaw + 180), 360);
        if(we_are_blue){
            gps_x = -gps_x;
            gps_y = -gps_y;
            gps_theta = package.yaw;
        }
        if (skills_gps_deadzone(gps_x, gps_y)) {
                pros::lcd::print(3, "gps dead"); // print the heading
                std::cout<<"gps dead"<<std::endl; //gps out of range
        }
        else {pros::lcd::clear_line(3);}
        pros::lcd::print(4, "GPS's x: %f", gps_x); // print the x position
        pros::lcd::print(5, "GPS's y: %f", gps_y); // print the y position
        pros::lcd::print(6, "GPS's heading: %f", gps_theta); // print the heading

        // ---------------- Odometry ---------------- //
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", fmod(pose.theta,360)); // print the heading

        // pros::lcd::print(8, "speed: %f degrees/s", (turnTable.get_value()-prev_turn_table_tick)/0.01); // speed
        // prev_turn_table_tick = turnTable.get_value();

        pros::delay(10);
    }
}


void match_near(){
    chassis.setPose(-52.1,-56.6,242.2);
    Intake.move_voltage(-12000);
    //scoop
    intake_out();
    wings_out();
    pros::delay(500);
    wings_in();
    chassis.turnTo(-3, -62,2000);
    //butt it in
    chassis.moveTo(-60, 0, 180, 2000, false, false);
    //come back
    chassis.moveTo(-29, -35, 0, 2000);
    wings_out();
    //push ball over
    chassis.moveTo(-6, -11, 90, 2000);
    wings_in();
    //come back diagonally
    chassis.moveTo(-54, -54, 45, 2000, false, false);
    //drive to deliver
    chassis.moveTo(-12, -62, 90, 2000);
    intake_in();
}

void match_far(){
    chassis.setPose(38.5,-53.6,341.3);
    //fling the preload 
    wings_out();
    //get ball 1
    intake_out();
    Intake.move_voltage(12000);
    pros::delay(500);
    wings_in();
    chassis.moveTo(20, -7, 341.3, 2000);
    pros::delay(500);
    //outake ball 1
    chassis.turnTo(47, -11, 2000);
    Intake.move_voltage(-12000);
    pros::delay(500);
    //get ball 2
    chassis.turnTo(5, -1, 2000);
    Intake.move_voltage(12000);
    chassis.moveTo(10, -4, 285, 2000);
    pros::delay(500);
    //outake ball 2
    chassis.turnTo(47, -11, 2000);
    Intake.move_voltage(-12000);
    pros::delay(500);
    //get ball 3
    Intake.move_voltage(12000);
    chassis.moveTo(7, -21, 220, 2000);
    pros::delay(500);
    //outake ball 3 + push w/ wings
    chassis.turnTo(47, -11, 2000);
    Intake.move_voltage(-12000);
    wings_out();
    intake_in();
    chassis.moveTo(37, -6, 90, 1500);
    //come back
    wings_in();
    Intake.brake();
    chassis.moveTo(44, -46, 225, 2000, false, false);
    chassis.moveTo(60, -24, 180, 2000, false, false);
}

void skills(){
    chassis.setPose(-49,55,22);
    //push 2 red balls
    chassis.moveTo(-55, 25, 0, 1500,false,false, 20);
    //move to match load
    chassis.moveTo(-55, 43, 0, 1500);
    chassis.turnTo(48, 10, 1500,false,true);
    wings_out();
    //match load for 25 secs
    Cata.move_velocity(53);
    pros::delay(2000);
    Cata.brake();
    wings_in();
    //get corner ball
    chassis.moveTo(-22, 40, 270, 2000, false, false);
    right_back_wing_out();
    left_back_wing_out();
    //sweeps
    chassis.moveTo(-14, 24, 0, 2000, false, false);
    //push balls over
    chassis.moveTo(-16, -40, 20, 2000, false, false,20);
    left_back_wing_in();
    //bend the nook
    chassis.turnTo(-62, -42, 1000,false,true);
    chassis.moveTo(-66, -35, 90, 1000, false, false);
    //get the corner ball
    chassis.turnTo(-53, -53, 1000);
    right_back_wing_out();
    chassis.moveTo(-45, -59, 135, 2000);
    chassis.turnTo(47, -57, 1000);
    //get to other side
    chassis.moveTo(-21, -58, 90, 2000);
    chassis.moveTo(42, -58, 90, 2000);
    chassis.moveTo(-38, -60, 270, 2000, false, false);
    right_back_wing_out();
    chassis.moveTo(-5, -60, 270, 2000, false, false);
    right_back_wing_in();
    chassis.moveTo(7, -60, 270, 2000, false, false);
    right_back_wing_out();
    chassis.moveTo(42, -60, 270, 2000, false, false);
    //get ball in corner
    chassis.moveTo(53, -53, 225, 2000, false, false);
    left_back_wing_out();
    right_back_wing_in();
    //push from side #1
    chassis.moveTo(59, -25, 180, 2000, false, false);
    left_back_wing_in();
    //back off
    chassis.moveTo(59, -40, 180, 2000);
    //turn to center
    chassis.turnTo(11, -31, 100, false, true);
    //go inside the nook
    chassis.moveTo(11, -29, 117, 2000,false,false);
    //go along the pipe to the middle
    chassis.moveTo(5, -6, 180, 2000,false,false);
    //turn & push 1
    chassis.turnTo(70, 0, 1500,false,false,80);
    chassis.moveTo(72, 0, 90, 1500);
    //back out
    chassis.moveTo(5, -6, 90, 2000, false,false);
    chassis.moveTo(5, 30, 180, 2000, false,false);
    chassis.moveTo(5, 30, 135, 2000, false,false);
    //push 2
    wings_out();
    chassis.moveTo(48, 3, 90, 2000);
    //back out
    wings_in();
    chassis.moveTo(5, -6, 90, 2000, false,false);
    chassis.moveTo(5, -30, 0, 2000, false,false);
    chassis.moveTo(5, -30, 45, 2000, false,false);
    //push 3
    wings_out();
    chassis.moveTo(48, -4, 90, 2000);
    //back out
    wings_in();
    chassis.moveTo(5, -6, 90, 2000, false,false);
    //push 4 - remaining balls
    wings_out();
    chassis.moveTo(72, 0, 90, 2000);
    //back out
    wings_in();
}

void initialize() {
	pros::lcd::initialize();
	chassis.calibrate();
	pros::Task screenTask(screen); // create a task to print the position to the screen
	Left.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    Right.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    Cata.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    Intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	// //Auto function
	// chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
}

void disabled() {
}

void competition_initialize() {}

void autonomous() {
	// chassis.turnTo(53, 53, 1000); // turn to the point (53, 53) with a timeout of 1000 ms
    // chassis.turnTo(-20, 32, 1500, true); // turn to the point (-20, 32) with the back of the robot facing the point, and a timeout of 1500 ms
    // chassis.turnTo(10, 0, 1000, false, 50); // turn to the point (10, 0) with a timeout of 1000 ms, and a maximum speed of 50

	// chassis.moveTo(53, 53, 1000); // move to the point (53, 53) with a timeout of 1000 ms
    // chassis.moveTo(10, 0, 1000, 50); // move to the point (10, 0) with a timeout of 1000 ms, and a maximum speed of 50
	
    //chassis.setPose(0,0,0);
    // chassis.moveTo(0, 30,0,2000);
    //chassis.moveTo(0, 0, 90,1000);

	//chassis.turnTo(30, 0, 1000);
	// std::cout<<Gyro.get_heading()<<std::endl;


	// file name: path.txt
    // timeout: 2000 ms
    // lookahead distance: 15 inches
    //chassis.follow("path.txt", 100000, 15);
    //std::cout<<"done"<<std::endl;
    // follow the next path, but with the robot going backwards
    //chassis.follow("path2.txt", 2000, 15, true);

    //chassis.moveTo(0, 30, 0, 2000);
    //chassis.turnTo(30,0, 2000);
    
    //match_near();

    skills();

    //match_near();
}

void opcontrol() {
    bool rapid_fire = false;
    // ---------------- Drivetrain ---------------- //
    double turnImportance = 0.3; //How much turning slows down the speed of forward, 0 doesn't affect, 1 stops forward
    double turnSensitivity = 0.6; //How sensitive a turn is, 0 doesn't turn, 1 most sensitive

    while(true){
        // ---------------- Drivetrain ---------------- //
        double motorTurnVal = Controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)/(double)127*100; //convert to percentage
        double motorForwardVal = Controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)/(double)127*100; //convert to percentage
        //Volts range: -12 --> 12, converts percentage to volts
        double motorTurnVolts = turnSensitivity*(motorTurnVal * 0.12);
        //Times forward volts by a percentage from how much you turn and how important the turn is to slowing down forward speed
        double motorForwardVolts = motorForwardVal * 0.12 * (1 - (std::abs(motorTurnVolts)/12 * turnImportance)); 
        Left.move_voltage ((motorForwardVolts + motorTurnVolts) * (double) 1000);
        Right.move_voltage ((motorForwardVolts - motorTurnVolts) * (double) 1000);
        //std::cout<<LeftFront.get_actual_velocity()<<std::endl;

        // ---------------- Catapult ---------------- //
        //variables init
        if(Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) rapid_fire = !rapid_fire;

        //Buttons
        if(rapid_fire) Cata.move_velocity(55);
        else if(Controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) Cata.move_velocity(55);
        else set_cata(down_ang);

        // ---------------- Intake ---------------- //
        if(Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            Intake.move_voltage(12000);
        }
        else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            Intake.move_voltage(-12000);
        }
        else Intake.brake();
        if(Controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            intake_out();
        }
        else if(Controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            intake_in();
        }

        // ---------------- Wings ---------------- //
        if(Controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            wings_out();
        }
        else if(Controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            wings_in();
        }

        // ---------------- Back Wings ---------------- //
        if(Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            if(right_back_wing)right_back_wing_in();
            else right_back_wing_out();
            right_back_wing = !right_back_wing; //toggle
        }
        else if(Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
            if(left_back_wing)left_back_wing_in();
            else left_back_wing_out();
            left_back_wing = !left_back_wing; //toggle
        }

        pros::delay(2);
  }
}
