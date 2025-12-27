#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/odom.hpp"
#include "lemlib/pose.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <chrono>
#include <cstdio>
#include <iostream>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-7, -18, -16},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({19, 12, 13}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(21);


// tracking wheels
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(8);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -1);

// Other motors and sensors

pros::adi::DigitalOut matchloader('G',LOW);
pros::Motor Intake(10 ,pros::MotorGearset::blue);
pros::Optical ColorSort(9);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(15, // proportional gain (kP)
                                            -0.1, // integral gain (kI)
                                            100, // derivative gain (kD)
                                            3, // anti windup 3
                                            1, // small error range, in inches 1
                                            100, // small error range timeout, in milliseconds 100
                                            2, // large error range, in inches 3
                                            500, // large error range timeout, in milliseconds 500
                                            0 // maximum acceleration (slew) 20
);

// angular motion controller
lemlib::ControllerSettings angularController(1.8, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             11.5, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
//variables
bool backVal = false;
float derivative;
bool slapVal = false;
int state=0;
int IntakeVel=0;
bool auton=false;
int error;


//Color Sort
bool colorsortRED=true;
bool colorsortBLUE=false;
bool intakeRun = true;
int intakeStuckTime=0;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    pros::Task ColorSorter([&]() {
        while(true){
            if (colorsortRED==true){
                if ((ColorSort.get_hue()>300 || ColorSort.get_hue()<20) && ColorSort.get_proximity()>100){
                    pros::c::delay(192);
                    intakeRun=false;
                    IntakeVel=127;
                    Intake.move(127);
                    pros::c::delay(280);
                    intakeRun=true;
                    IntakeVel=-127;
                    Intake.move(-127);
                }
            }
            if (colorsortBLUE==true) {
                if (ColorSort.get_hue()>150 && ColorSort.get_proximity()>100){
                    if (ColorSort.get_hue()<220){
                        pros::c::delay(185);
                        intakeRun=false;
                        IntakeVel=127;
                        Intake.move(127);
                        pros::c::delay(280);
                        intakeRun=true;
                        IntakeVel=-127;
                        Intake.move(-127);
                    }
                }
            }
        }
        pros::delay(50);
    });
   
    pros::Task intakeStuckDetect([&]() {
        while (true){
            Intake.move(IntakeVel);
            if (auton==true){
                if (Intake.get_actual_velocity()==0 && abs(IntakeVel)>60){
                    intakeStuckTime = intakeStuckTime+50;
                } else{
                    intakeStuckTime=0;
                }
                if (intakeStuckTime>400){
                    IntakeVel=127;
                    Intake.move(127);
                    pros::c::delay(500);
                    IntakeVel=-127;                
                    Intake.move(-127);
                }
            }

            pros::c::delay(50);
        }
});

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    /*pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });*/
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

void blueneg(){
    IntakeVel=0;
    colorsortRED=true;
    colorsortBLUE=false;
    matchloader.set_value(false);
    ColorSort.set_led_pwm(100);
    auton=true;
    chassis.setPose(15.6,12.1,235.2);
    state=4;
    pros::c::delay(500);
    chassis.moveToPose(23, 47, 170, 2000,{.forwards = false,.maxSpeed=127,.minSpeed=52});
    pros::delay(100);
    state=0;
    chassis.waitUntilDone();
    pros::c::delay(300);
    matchloader.set_value(true);
    chassis.turnToHeading(50,1000);
    chassis.waitUntilDone();
    IntakeVel=-127;
    Intake.move(IntakeVel);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    //version 1: stop at 1st ring
    chassis.moveToPose(45, 63.25, 90, 1500,{.maxSpeed=62,.minSpeed=42});
    chassis.waitUntilDone();
    pros::delay(700);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveToPoint(53, 63.7, 3000,{.maxSpeed=42,.minSpeed=36});
    chassis.waitUntilDone();
    pros::c::delay(700);
    chassis.turnToPoint(48, 48, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(48, 48, 1500,{.maxSpeed=82,.minSpeed=52});
    chassis.waitUntilDone();
    pros::delay(900);
    chassis.turnToPoint(70, 4, 1000);
    //chassis.moveToPose(64.2, 0.9, 167, 3000,{.maxSpeed=82,.minSpeed=57});
    chassis.moveToPose(66, 3, 167, 3000,{.maxSpeed=82,.minSpeed=57});
    pros::c::delay(2500);
    chassis.moveToPose(52, 11, 167, 2000,{.forwards=false,.maxSpeed=62,.minSpeed=36});
    chassis.waitUntilDone();
    chassis.moveToPose(54, 10, 167, 2000,{.maxSpeed=62,.minSpeed=36});
    pros::delay(900);
    chassis.moveToPose(-48,16,90,2000,{.forwards=false,.maxSpeed=127,.minSpeed=100,.earlyExitRange=4});
    chassis.waitUntilDone();
}

void redneg(){
    IntakeVel=0;
    colorsortRED=false;
    colorsortBLUE=true;
    matchloader.set_value(false);
    ColorSort.set_led_pwm(100);
    auton=true;
    chassis.setPose(-15.6,12.1,124.8);
    state=4;
    pros::c::delay(500);
    chassis.moveToPose(-23, 47, 190, 2000,{.forwards = false,.maxSpeed=127,.minSpeed=52});
    pros::delay(100);
    state=0;
    chassis.waitUntilDone();
    pros::c::delay(300);
    matchloader.set_value(true);
    chassis.turnToHeading(310,1000);
    chassis.waitUntilDone();
    IntakeVel=-127;
    Intake.move(IntakeVel);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    //version 1: stop at 1st ring
    chassis.moveToPose(-45, 63.05, 270, 1500,{.maxSpeed=62,.minSpeed=42});
    chassis.waitUntilDone();
    pros::delay(700);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveToPoint(-53, 63.35, 3000,{.maxSpeed=42,.minSpeed=36});
    chassis.waitUntilDone();
    pros::c::delay(700);
    chassis.turnToPoint(-48, 48, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-48, 48, 1500,{.maxSpeed=82,.minSpeed=52});
    chassis.waitUntilDone();
    pros::delay(900);
    chassis.turnToPoint(-70, 4, 1000);
    //chassis.moveToPose(64.2, 0.9, 167, 3000,{.maxSpeed=82,.minSpeed=57});
    chassis.moveToPose(-66.3, 2.4, 193, 3000,{.maxSpeed=82,.minSpeed=57});
    pros::c::delay(2500);
    chassis.moveToPose(52, 11, 193, 2000,{.forwards=false,.maxSpeed=62,.minSpeed=36});
    chassis.waitUntilDone();
    chassis.moveToPose(54, 10, 193, 2000,{.maxSpeed=62,.minSpeed=36});
    pros::delay(900);
    chassis.moveToPose(-48,16,270,2000,{.forwards=false,.maxSpeed=127,.minSpeed=100,.earlyExitRange=4});
    chassis.waitUntilDone();
}

void redpos(){
    matchloader.set_value(false);
    colorsortRED=false;
    colorsortBLUE=true;
    auton=true;
    matchloader.set_value(false);
    IntakeVel=0;
    chassis.setPose(15.6,12.1,235.2);
    //state=4;
    //pros::c::delay(500);
    chassis.moveToPose(23, 47, 170, 2000,{.forwards = false,.maxSpeed=127,.minSpeed=52});
    //pros::c::delay(100);
    //state=0;
    chassis.waitUntilDone();
    pros::delay(200);
    matchloader.set_value(true);
    pros::delay(500);
    IntakeVel=-127;
    pros::c::delay(500);
    chassis.turnToPoint(47.5, 47, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(41, 47, 1000,{.maxSpeed=124,.minSpeed=54});
    chassis.waitUntilDone();
    pros::delay(900);
    chassis.moveToPose(62, 24, 150, 2000,{.maxSpeed=86,.minSpeed=42});
    chassis.waitUntilDone();
    chassis.moveToPoint(63, 4, 2000,{.maxSpeed=52,.minSpeed=30});
    chassis.waitUntilDone();
    pros::delay(1500);
    chassis.moveToPose(48, 18, 115, 2000,{.forwards=false,.maxSpeed=82,.minSpeed=32});
    chassis.waitUntilDone();
    chassis.moveToPose(51, 14, 115, 2000,{.maxSpeed=82,.minSpeed=32});
    pros::delay(500);
    chassis.moveToPoint(3, 61, 2000,{.forwards=false,.maxSpeed=84});
    state=3;
}

void bluepos(){
    matchloader.set_value(false);
    colorsortRED=true;
    colorsortBLUE=false;
    auton=true;
    IntakeVel=0;
    matchloader.set_value(false);
    chassis.setPose(-15.6,12.1,124.8);
    state=4;
    pros::c::delay(500);
    chassis.moveToPose(-23, 47, 190, 2000,{.forwards = false,.maxSpeed=127,.minSpeed=50});
    pros::c::delay(100);
    state=0;
    chassis.waitUntilDone();
    pros::delay(200);
    matchloader.set_value(true);
    pros::delay(500);
    IntakeVel=-127;
    pros::c::delay(500);
    chassis.turnToPoint(-47.5, 47, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-41, 47, 1000,{.maxSpeed=124,.minSpeed=54});
    chassis.waitUntilDone();
    pros::delay(900);
    chassis.moveToPose(-62, 24, 210, 2000,{.maxSpeed=86,.minSpeed=42});
    chassis.waitUntilDone();
    chassis.moveToPoint(-63, 4, 2000,{.maxSpeed=52,.minSpeed=30});
    chassis.waitUntilDone();
    pros::delay(1500);
    chassis.moveToPose(-48, 18, 245, 2000,{.forwards=false,.maxSpeed=82,.minSpeed=32});
    chassis.waitUntilDone();
    chassis.moveToPose(-51, 14, 245, 2000,{.maxSpeed=82,.minSpeed=32});
    pros::delay(500);
    chassis.moveToPoint(-3, 61, 2000,{.forwards=false,.maxSpeed=84});
    state=3;
}

void skills(){
    //alliance + clamp goal 1
    colorsortRED=false;
    colorsortBLUE=false;
    auton=true;
    chassis.setPose(-15.6,11.1,124.8);
    state=4;
    pros::delay(600);
    chassis.moveToPose(-24, 24, 145, 1100,{.forwards=false,.maxSpeed=84,.minSpeed=42});
    pros::delay(100);
    state=0;
    chassis.waitUntilDone();
    pros::delay(300);
    .set_value(true);
    // rings on goal 1 + wall stake
    pros::delay(500);
    chassis.turnToHeading(0,500);
    chassis.waitUntilDone();
    IntakeVel=-127;
    chassis.moveToPose(-24, 48, 0, 800, {.maxSpeed=82,.minSpeed=52});
    chassis.waitUntilDone();
    chassis.turnToHeading(290, 1000);
    IntakeVel=-127;
    chassis.waitUntilDone();
    chassis.moveToPose(-60,71.5,280,1500, {.maxSpeed=82,.minSpeed=54});
    chassis.waitUntilDone();
    IntakeVel=-95;
    state=1;
    pros::delay(500);
    chassis.moveToPose(-65.7, 71.8, 290, 800, {.maxSpeed=52,.minSpeed=32});
    pros::delay(700);
    IntakeVel=0;
    pros::delay(300);
    IntakeVel=-127;
    pros::delay(200);
    IntakeVel=0;
    pros::delay(100);
    chassis.waitUntilDone();
    chassis.turnToPoint(-77, 74.6, 400);
    chassis.waitUntilDone();
    chassis.moveToPoint(-66.5, 72.3, 700,{.maxSpeed=42,.minSpeed=42});
    chassis.waitUntilDone();
    state=4;
    pros::c::delay(500);
    chassis.moveToPose(-48, 70, 270, 600,{.forwards=false,.maxSpeed=82,.minSpeed=52});
    pros::c::delay(300);
    state=0;
    IntakeVel=-127;
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.turnToHeading(180, 500);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveToPoint(-48,24, 1600, {.maxSpeed=50,.minSpeed=32});
    chassis.waitUntilDone();
    pros::delay(400);
    chassis.moveToPoint(-48, 12, 600,{true,42,32});
    chassis.waitUntilDone();
    pros::delay(400);
    chassis.turnToPoint(-60, 24, 800);
    chassis.waitUntilDone();
    chassis.moveToPoint(-60, 24, 1000,{.maxSpeed=52,.minSpeed=42});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.turnToHeading(20, 800);
    chassis.waitUntilDone();
    chassis.moveToPose(-66, 6, 20, 900,{.forwards=false,.minSpeed=56});
    chassis.waitUntilDone();
    IntakeVel=127;
    pros::delay(200);
    matchloader.set_value(false);
    chassis.moveToPoint(-48, 24, 1000,{.maxSpeed=52});
    chassis.turnToHeading(270, 900);
    chassis.waitUntilDone();
    //driving to goal 2
    chassis.moveToPose(0, 24, 270, 1500,{.forwards=false,.maxSpeed=94,.minSpeed=62});
    chassis.waitUntilDone();
    chassis.moveToPoint(24, 23, 1000,{.forwards=false,.maxSpeed=54,.minSpeed=42});
    chassis.waitUntilDone();
    matchloader.set_value(true);
    //rings on goal 2
    pros::delay(500);
    chassis.turnToHeading(0,800);
    chassis.waitUntilDone();
    IntakeVel=-127;
    chassis.moveToPose(24, 48, 0, 900, {.maxSpeed=82,.minSpeed=52});
    chassis.waitUntilDone();
    chassis.turnToHeading(80, 900);
    chassis.waitUntilDone();
    IntakeVel=-127;
    chassis.moveToPose(60,71.5,80,700, {.maxSpeed=82,.minSpeed=54});
    chassis.waitUntilDone();
    IntakeVel=-95;
    state=1; //stops around here, intake and lady brown keep running but does not drive
    pros::delay(500);
    chassis.moveToPose(65.7, 71.8, 70, 1500, {.maxSpeed=62,.minSpeed=32});
    pros::delay(700);
    IntakeVel=0;
    pros::delay(300);
    IntakeVel=-127;
    pros::delay(200);
    IntakeVel=0;    
    pros::delay(100);
    chassis.turnToPoint(77, 74.6, 600);
    chassis.waitUntilDone();
    chassis.moveToPoint(66.5, 72.3, 900,{.maxSpeed=42,.minSpeed=42});
    chassis.waitUntilDone();
    state=4;
    pros::c::delay(500);
    chassis.moveToPose(48, 70, 90, 800,{.forwards=false,.maxSpeed=82,.minSpeed=52});
    pros::c::delay(900);
    state=0;
    IntakeVel=-127;
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.turnToHeading(180, 1000);
    chassis.moveToPoint(48,27, 1700, {.maxSpeed=50,.minSpeed=32});
    chassis.waitUntilDone();
    pros::delay(400);
    chassis.moveToPoint(48, 12, 700,{true,42,32});
    chassis.waitUntilDone();
    pros::delay(400);
    chassis.turnToPoint(60, 24, 500);
    chassis.waitUntilDone();
    chassis.moveToPoint(60, 24, 800,{.maxSpeed=52,.minSpeed=42});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.turnToHeading(340, 500);
    chassis.waitUntilDone();
    chassis.moveToPose(66, 6, 340, 800,{.forwards=false,.minSpeed=56});
    chassis.waitUntilDone();
    IntakeVel=127;
    pros::delay(200);
    matchloader.set_value(false);
    chassis.moveToPoint(48, 24, 800,{.maxSpeed=52});
    //driving to goal 3
    chassis.turnToPoint(48,72,600);
    chassis.waitUntilDone();
    chassis.moveToPose(48, 72, 330, 1200,{.maxSpeed=82,.minSpeed=70});
    chassis.waitUntilDone();
    IntakeVel=-92;
    chassis.moveToPose(24, 96, 315, 1000,{.maxSpeed=82,.minSpeed=60});
    chassis.waitUntilDone();
    pros::delay(200);
    IntakeVel=0;
    chassis.turnToHeading(120, 500);
    chassis.waitUntilDone();
    chassis.moveToPose(0, 120.1, 120, 1000,{.forwards=false,.maxSpeed=72,.minSpeed=42});
    chassis.waitUntilDone();
    pros::delay(300);
    matchloader.set_value(true);
    IntakeVel=30;
    pros::delay(100);
    //rings on goal 3
    IntakeVel=-127;
    pros::delay(500);
    chassis.moveToPose(24, 96, 135, 1000,{.maxSpeed=92,.minSpeed=52});
    chassis.waitUntilDone();
    chassis.moveToPose(0, 72, 225, 1000,{.maxSpeed=92,.minSpeed=52});
    chassis.waitUntilDone();
    chassis.moveToPose(-24, 96, 315, 1000,{.maxSpeed=92,.minSpeed=42});
    chassis.waitUntilDone();
    chassis.turnToHeading(270, 800);
    chassis.waitUntilDone();
    matchloader.set_value(true);
    pros::c::delay(200);
    IntakeVel=-127;
    chassis.moveToPose(-48, 96, 270, 1000,{.maxSpeed=92,.minSpeed=52});
    chassis.waitUntilDone();
    pros::delay(250);
    chassis.turnToPoint(-60, 120, 500);
    chassis.waitUntilDone();
    colorsortBLUE=true;
    chassis.moveToPoint(-60, 115, 900,{.maxSpeed=52});
    chassis.waitUntilDone();
    pros::delay(800);
    IntakeVel=0;
    chassis.moveToPose(-48, 132, 90, 700,{.maxSpeed=62,.minSpeed=42});
    chassis.waitUntilDone();
    chassis.turnToPoint(-24, 120, 500);
    chassis.waitUntilDone();
    chassis.moveToPoint(-60, 132, 800,{.forwards=false,.maxSpeed=52});
    chassis.waitUntilDone();
    IntakeVel=127;
    pros::delay(100);
    matchloader.set_value(false);
    pros::delay(100);
    chassis.moveToPose(-48, 130, 115, 700,{.maxSpeed=84,.minSpeed=42});
    //goal 4
    chassis.moveToPose(-24, 118, 90, 800,{.maxSpeed=127,.minSpeed=74,.earlyExitRange=4});
    pros::delay(300);
    IntakeVel=-127;
    chassis.waitUntilDone();
    chassis.moveToPose(24, 133, 60, 1000,{.maxSpeed=127,.minSpeed=74});
    chassis.waitUntilDone();
    chassis.moveToPoint(60, 135, 1000,{.maxSpeed=120,.minSpeed=64});
}
/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {   //alliance + clamp goal 1
    skills();
}

/**
 * Runs in driver control
 */

void opcontrol() {
    // controller
    // loop to continuously update motors;
    while (true) {
        auton=true;
        // get joystick positions
        colorsortRED=false;
        colorsortBLUE=false;
        ColorSort.set_led_pwm(100);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
            backVal = !backVal;
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            slapVal= !slapVal;
        }
        if (intakeRun==true){
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
                IntakeVel=127;
            }
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
                IntakeVel=-127;
            } else {
                Intake.brake();
                IntakeVel=0;
            }
        }
 

        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        matchloader.set_value(backVal);
        Intake.move(IntakeVel);

        
        // delay to save resources
        matchloaderset_value(slapVal);;
        pros::delay(10);
    }
}
