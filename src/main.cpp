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
    chassis.setPose(46,55,168);
    //scoop
    left_back_wing_out();
    chassis.turnTo(68, 25,2000);
    left_back_wing_in();
    //front it in
    chassis.moveTo(61, 22, 180, 1500);
    // //come back
    // chassis.moveTo(-29, -35, 0, 2000);
    // wings_out();
    // //push ball over
    // chassis.moveTo(-6, -11, 90, 2000);
    // wings_in();
    // //come back diagonally
    // chassis.moveTo(-54, -54, 45, 2000, false, false);
    // //drive to deliver
    // chassis.moveTo(-12, -62, 90, 2000); //6 60 270
    // intake_in();
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
    chassis.moveTo(12, -11, 317, 2000);
    pros::delay(500);
    //outake ball 1
    chassis.turnTo(47, -5, 2000);
    Intake.move_voltage(-12000);
    wings_out();
    //score ball 1 and 2
    chassis.moveTo(45, -5, 90, 1000);
    //get ball 3
    wings_in();
    chassis.moveTo(38, -7, 90, 2000, false, false);
    Intake.move_voltage(12000);
    chassis.moveTo(13, -18, 230, 2000);
    pros::delay(500);
    //outake ball 3 + push w/ wings
    chassis.turnTo(47, -11, 2000);
    Intake.move_voltage(-12000);
    wings_out();
    intake_in();
    chassis.moveTo(45, -6, 90, 1000);
    //come back
    wings_in();
    Intake.brake();
    chassis.moveTo(44, -46, 300, 2000, false, false);
    chassis.moveTo(56, -46, 270, 2000, false, false);
    left_back_wing_out();
    //get corner ball
    chassis.turnTo(68, -20, 2000, false, true);
    left_back_wing_in();
    chassis.moveTo(63, -24, 180, 1000, false, false);
    //leave
    chassis.moveTo(63, -30, 180, 2000);
}

void skills(){
    chassis.setPose(-50,55,24);
    //push 2 red balls
    chassis.moveTo(-57, 25, 0, 1000,false,false, 20);
    //move to match load
    chassis.moveTo(-56, 44, 0, 1500);
    chassis.moveTo(-56, 44, 285, 1000);
    wings_out();
    //match load for 25 secs
    Cata.move_velocity(52);
    pros::delay(2000);
    Cata.brake();
    wings_in();
    //head into the corridor
    chassis.moveTo(-40, 55, 40, 1500);
    //to other side
    chassis.moveTo(43, 62, 100, 2000);
    //crazy turning push! #1
    chassis.moveTo(65, 23, 180, 1500, false, true, 20);
    // //backoff for 2nd
    // chassis.moveTo(63, 50, 180, 2000, false, false);
    // //crazy side push! #2
    // chassis.moveTo(63, 25, 180, 1000);
    //turn and back comes to group balls diagonal
    chassis.moveTo(35, 45, 0, 2000, false, false);
    left_back_wing_out();
    //push balls closer to middle
    chassis.moveTo(35, 25, 0, 2000, false, false);
    chassis.moveTo(35, 32, 0, 2000);
    //comes back against black barrier
    left_back_wing_in();
    chassis.moveTo(13, 24, 90, 2000, false, false);
    //push #3!
    chassis.moveTo(43, 14, 100, 1500, false, true, 20);
    //come back against barrier
    chassis.moveTo(13, 24, 90, 2000, false, false);
    //turn towards middle
    chassis.turnTo(11, 0, 1000);
    //goes to middle
    chassis.moveTo(11, 0, 180, 2000, false, true, 0, 0.6,70);
    //turns to double push #4!
    chassis.turnTo(48, 0, 1000);
    wings_out();
    chassis.moveTo(56, 0, 90, 2000);
    chassis.moveTo(25, 0, 90, 2000, false, false);
    chassis.moveTo(56, 0, 90, 2000);
    //back off
    wings_in();
    chassis.moveTo(12, 9, 133, 2000, false, false);
    //turns toward side
    chassis.turnTo(11, -8, 1000);
    chassis.moveTo(11, -8, 180, 1000);
    //turns to push #5!
    chassis.turnTo(48, -8, 1000);
    wings_out();
    chassis.moveTo(56, -8, 90, 2000);
    //back off
    chassis.moveTo(11, -8, 90, 2000, false, false);
    wings_in();
    //turns toward edge
    chassis.turnTo(11, -31, 1000);
    chassis.moveTo(11, -31, 180, 2000, false, true, 0, 0.6,70);
    //turns toward middle
    chassis.turnTo(23, -21, 1000);
    wings_out();
    chassis.moveTo(23, -21, 55, 2000);
    //push #6!
    chassis.turnTo(47, -7, 1000);
    chassis.moveTo(62, -7, 90, 2000);
    //back off
    wings_in();
    right_back_wing_out();
    chassis.moveTo(26, -29, 330, 2000, false, false);
    chassis.moveTo(37, -44, 300, 2000, false, false);
    right_back_wing_in();
    //push #7!
    chassis.moveTo(62, -23, 180, 2000, false, false);
    //backoff for 2nd
    chassis.moveTo(39, -58, 231, 2000);
    right_back_wing_out();
    //crazy side push! #8
    chassis.moveTo(63, -23, 180, 2000, false, false);
    //final backoff
    chassis.moveTo(63, -50, 180, 2000);
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
    
    match_near();

    //skills();

    //match_far();
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
        if(rapid_fire) Cata.move_velocity(52);
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
