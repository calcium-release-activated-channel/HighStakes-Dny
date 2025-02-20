#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include <cstddef>


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
//CHANGE LATER
pros::MotorGroup leftMotors({-5, -11, 18},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({13, 19, -9}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)


// Inertial Sensor on port 10
pros::Imu imu(21); 

//horiontal odom wheel
pros::adi::Encoder horizontal_encoder('G', 'H', true); //check if need be reversed
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_325, -3); //POSSIBLY CHANGE DISTANCE IDFK


//lady brown code

//array to store 4 positions
//while loop and getting to target value
//maybe pid but probably not if using target value cause potentiometer is absolute





//define color sensor
//pros::Optical optical_sensor(20); //change port #

/*\
optical_sensor.set_led_pwm(100); // Set LED brightness to 100%
// Get the color detected
pros::c::optical_rgb_s_t color = optical_sensor.get_rgb();

printf("Red: %f, Green: %f, Blue: %f\n", color.red, color.green, color.blue);

or
pros::Optical::hue hue = optical_sensor.get_hue();
printf("Hue: %f\n", hue);

void colorSortRed() {
    if (optical_sensor.get_hue() > 330 && optical_sensor.get_hue() < 30) { //if red
    pros::delay(200);
    conveyor.move_voltage(-100);
    pros::delay(200);

    } else {
    conveyor.move_voltage(127); 
    }

    pros::delay(10);
}//end of colorSortRed

void colorSortBlue() {
    if(optical_sensor.get_hue() > 180 && optical_sensor.get_hue() < 300) {
    pros::delay(200);
    conveyor.move_voltage(-100);
    pros::delay(200);
    } else {
    conveyor.move_voltage(127);
    }
     pros::delay(10);
}

*/



// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.75, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_tracking_wheel, // horizontal tracking wheel
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



//SETTING UP NEW MOTORS AND PNUEMATICS AND OTHER STUFF-
//drive motors defined above

//intake
pros::Motor intake(-15);
pros::Motor conveyor(-14);


//pnuematics (doinker/mogo)
pros::adi::Pneumatics mogo('A', false);
pros::adi::Pneumatics doinker('B', false);

//ladyBrown + lb potentiomter
pros::Motor ladyBrown(-1); //single motor check if needed to be reversed and port # (assuming positive is forwards toward intake)
pros::adi::Potentiometer lbSensor('E'); //range of 270* get proper measurements i want
//passive = up, potentiometer value = 
//loading = right, potentiometer value = 
//scoring = left, potentiometer value = 
//going down = down, potentiometer value = 



void drive() {

        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the chassis with curvature drive
      //  chassis.arcade(leftY, rightX);
		chassis.tank(leftY, rightY);
        // delay to save resources
        pros::delay(10);
    
}



bool extendPiston = false;  // Declare outside the function for persistent state

void setMogo(bool extend) {
    mogo.set_value(extend);
   
}

void setMogoSolenoids() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        extendPiston = !extendPiston;  // Toggle the state
        setMogo(extendPiston);         // Apply the toggled state to pneumatics
        pros::delay(300);              // Delay to prevent multiple toggles from a single press
    }
    pros::delay(10);
}


//doinker
bool extendPiston2 = false;  // Declare outside the function for persistent state

void setDoinker(bool extend) {
    doinker.set_value(extend);
   
}

void setDoinkerSolenoids() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        extendPiston = !extendPiston;  // Toggle the state
        setDoinker(extendPiston);         // Apply the toggled state to pneumatics
        pros::delay(300);              // Delay to prevent multiple toggles from a single press
    }
    pros::delay(10);
}



void setIntake(int power) {  // intake power   //call this during auton.
    
    intake.move_voltage(power*1.2); //possibly change this value
    conveyor.move_voltage(power);
}

// driver controller functions
void setIntakeMotors() { //call this during opcontrol

    // bottom trigger intakes and top trigger outtakes
    // link belt to the same thigies

    int intakePower = 10000 * ((controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) - (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)));
   

    setIntake(intakePower);
   
}


//lady brown stuff
// Arm positions (potentiometer values)
const int ARM_POSITIONS[] = {3889, 3430, 1830}; // Passive, Loading, Scoring
const int NUM_POSITIONS = 3; // Total number of positions
int targetPosition = -1; // -1 means no macro is running
bool isManualControl = false; // Track manual movement
int currentPhase = 0; // Tracks which phase we're on (index in ARM_POSITIONS)

void moveArmToPosition(int index) {
    targetPosition = ARM_POSITIONS[index];
 } 

// Function to set arm power (Using Velocity Instead of Voltage)
void setLBPower(int power) {
    ladyBrown.move_velocity(power); // Uses velocity instead of voltage
}

// Background task for macros
void armControlTask(void* param) {
    while (true) {
        if (targetPosition != -1 && !isManualControl) { 
            int currentPos = lbSensor.get_value();
     //       int error = currentPos - targetPosition; // Flipped error calculation
            int error = targetPosition - currentPos;

            int power = (error > 0) ? -100 : 100; // Flipped movement direction

            // Slow down gradually when approaching target
            if (abs(error) < 300) power = (error > 0) ? -50 : 50;
            if (abs(error) < 150) power = (error > 0) ? -25 : 25;

            // Stop when inside the target range
            if (abs(error) < 50) {
                power = 0;
                targetPosition = -1; // Stop tracking macro once reached
            }

            setLBPower(power);
        }

        pros::delay(10); // Prevent CPU overuse
    }
}

// Main function to control Lady Brown
void setLadyBrown() {
    // Manual movement (X = Up, B = Down)
    bool xPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    bool bPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    
    int manualPower = 160 * (xPressed - bPressed); // Using max 100 velocity instead of voltage

    if (manualPower != 0) {
        targetPosition = -1; // Cancel macros when manually controlling
        isManualControl = true; // Track manual mode
        setLBPower(manualPower);
    } else if (isManualControl) { 
        // Stop movement when button is released
        isManualControl = false;
        setLBPower(0);
    }

    // **L1: Cycle Through Positions**
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) { //channge back to l1 possibly
        currentPhase = (currentPhase + 1) % NUM_POSITIONS; // Cycle through 0 → 1 → 2 → 0
        targetPosition = ARM_POSITIONS[currentPhase]; // Set target position
    }

    // Ensure macros actually set `targetPosition` properly
    if (!isManualControl) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            targetPosition = ARM_POSITIONS[0]; // Passive
        } 
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            targetPosition = ARM_POSITIONS[1]; // Loading
        } 
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            targetPosition = ARM_POSITIONS[2]; // Scoring
        }
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 */
void initialize() {
    // Set motors to brake mode
    ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // Start the arm control task
    pros::Task armTask(armControlTask);

    // Set other motor brake modes
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    pros::lcd::initialize(); // Initialize brain screen
    chassis.calibrate(); // Calibrate sensors

    // Thread for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(3, "ADI Encoder: %i", horizontal_encoder.get_value());
            pros::lcd::print(4, "LB sensor: %i", lbSensor.get_value());
            pros::lcd::print(5, "Target Pos: %i", targetPosition);
            pros::lcd::print(6, "Manual Mode: %i", isManualControl);
            pros::lcd::print(7, "Error: %i", lbSensor.get_value() - targetPosition);
            pros::lcd::print(8, "Current Phase: %i", currentPhase); // **Show which phase we're on**

            pros::delay(50);
        }
    });
}





/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
 /*
void initialize() {
    //set motors to coast
    
    //MOTOR STUFF
	ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //armTask = pros::Task(armControlTask, nullptr);
    pros::Task armTask(armControlTask);

    
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources

            // print measurements from the horizontal wheel
        pros::lcd::print(3, "ADI Encoder: %i", horizontal_encoder.get_value());
            pros::lcd::print(4, "LB sensor: %i", lbSensor.get_value());

            pros::delay(50);
        }
    });
}
*/

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {

}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */

// void blueNegative() {
//     //going to first mogo 
// chassis.setPose(58.772, 36.387, 90);
// chassis.moveToPoint(33.202, 36.387, 1000, {.forwards = false, .maxSpeed = 127}, true);
// chassis.swingToHeading(0, lemlib::DriveSide::LEFT, 1000, {}, true);
// chassis.moveToPose(24.108, 27.227,0,1000, {.forwards = false, .maxSpeed = 127}, false);
// setMogo(true);

// //reset
// }

void noAuton() {
//nothing happens
}

void moveAuton() {
    leftMotors.move_velocity(600);
    rightMotors.move_velocity(600);
    pros::delay(1000);
     leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
}

void blueNegativeOLD() {

//going to first mogo 
chassis.setPose(58.772, 36.387, 90);
chassis.moveToPoint(33.202, 36.387, 1000, {.forwards = false, .maxSpeed = 127}, true);
chassis.swingToHeading(0, lemlib::DriveSide::LEFT, 1000, {}, true);
chassis.moveToPose(24.108, 27.227,0,1000, {.forwards = false, .maxSpeed = 127}, false);
setMogo(true);



//turning towards middle stack and intaking 1
chassis.turnToPoint(9.701, 39.037, 1000, {.forwards = true, .maxSpeed = 127}, true);
setIntake(11000);
chassis.moveToPoint(9.701, 39.037, 1000, {.forwards = true, .maxSpeed = 127}, true);

//going backwards for single stack
chassis.moveToPoint(22.07, 32.003, 1000, {.forwards = false, .maxSpeed = 127}, true);

//going to single stack
chassis.turnToPoint(23.04, 41.219, 1000, {.forwards = true, .maxSpeed = 127}, true);
chassis.moveToPoint(23.04, 41.219, 1000, {.forwards = true, .maxSpeed = 127}, true);

//going to middle stack
chassis.turnToPoint(10.186, 48.01, 1000, {.forwards = true, .maxSpeed = 127}, true);
chassis.moveToPoint(10.186, 448.01, 1000, {.forwards = true, .maxSpeed = 127}, true);
pros::delay(200);
setIntake(0);

//going backwards 
chassis.moveToPose(36.141, 0.071, 0, 2000, {.forwards = false, .maxSpeed = 127}, true);
//dropping off mogo
chassis.moveToPose(43.402, -14.692, 155, 2000, {.forwards = false, .maxSpeed = 127}, true);
setMogo(false);

//going to new mogo and clamping
chassis.moveToPose(31.597, -20, 62, 2000, {.forwards = false, .maxSpeed = 127 }, true);
setMogo(true);

//going for ring
chassis.turnToPoint(26.2 ,-39.83, 1000, {.forwards = true, .maxSpeed = 127}, false);
setIntake(11000);
chassis.moveToPoint(26.2, -29.83, 1000, {.forwards = true, .maxSpeed = 127}, true);

//touch ladder
chassis.moveToPose(10.29, -26.62,315,  1000, {.forwards = true, .maxSpeed = 127}, false);
}



void redNegativeOld() {

//going to first mogo 
chassis.setPose(-58.772, 36.387, 270);
chassis.moveToPoint(-33.202, 36.387, 1000, {.forwards = false, .maxSpeed = 127}, true);
chassis.swingToHeading(180, lemlib::DriveSide::LEFT, 1000, {}, true);
chassis.moveToPose(-24.108, 27.227,180,1000, {.forwards = false, .maxSpeed = 127}, false);
setMogo(true);



//turning towards middle stack and intaking 1
chassis.turnToPoint(-9.701, 39.037, 1000, {.forwards = true, .maxSpeed = 127}, true);
setIntake(11000);
chassis.moveToPoint(-9.701, 39.037, 1000, {.forwards = true, .maxSpeed = 127}, true);

//going backwards for single stack
chassis.moveToPoint(-22.07, 32.003, 1000, {.forwards = false, .maxSpeed = 127}, true);

//going to single stack
chassis.turnToPoint(-23.04, 41.219, 1000, {.forwards = true, .maxSpeed = 127}, true);
chassis.moveToPoint(-23.04, 41.219, 1000, {.forwards = true, .maxSpeed = 127}, true);

//going to middle stack
chassis.turnToPoint(-10.186, 48.01, 1000, {.forwards = true, .maxSpeed = 127}, true);
chassis.moveToPoint(-10.186, 448.01, 1000, {.forwards = true, .maxSpeed = 127}, true);
pros::delay(200);
setIntake(0);

//going backwards 
chassis.moveToPose(-36.141, 0.071, 180, 2000, {.forwards = false, .maxSpeed = 127}, true);
//dropping off mogo
chassis.moveToPose(-43.402, -14.692, 25, 2000, {.forwards = false, .maxSpeed = 127}, true);
setMogo(false);

//going to new mogo and clamping
chassis.moveToPose(-31.597, -20, 118, 2000, {.forwards = false, .maxSpeed = 127 }, true);
setMogo(true);

//going for ring
chassis.turnToPoint(-26.2 ,-39.83, 1000, {.forwards = true, .maxSpeed = 127}, false);
setIntake(11000);
chassis.moveToPoint(-26.2, -29.83, 1000, {.forwards = true, .maxSpeed = 127}, true);

//touch ladder
chassis.moveToPose(-10.29, -26.62, 225,  1000, {.forwards = true, .maxSpeed = 127}, false);
}

void progSkills() {
    
    /*
    rightMotors.move(15);
    leftMotors.move(15);
    pros::delay(2000);
    rightMotors.move(0);
    leftMotors.move(0);
    */
    // final skills
   
    chassis.setPose(-58, 0, 90);
    setIntake(11000);
    pros::delay(1000);
    rightMotors.move(50);
    leftMotors.move(50);
    pros::delay(600);
    rightMotors.move(0);
    leftMotors.move(0);
    pros::delay(100);
    chassis.turnToPoint(-47.5, -26, 1000, {.forwards = false});
    chassis.moveToPoint(-47.5, -26, 1500, {.forwards = false, .maxSpeed=60});
    conveyor.move(0);
    pros::delay(850);
    setMogo(true);  
    chassis.turnToPoint(-58, -50, 1000);
    pros::delay(10);
    setIntake(11000);
    pros::delay(300);
    chassis.moveToPoint(-58, -50, 1500);
    pros::delay(1200);
    setIntake(11000);
    pros::delay(1000);
    rightMotors.move(-50);
    leftMotors.move(-50);
    pros::delay(200);
    rightMotors.move(0);
    leftMotors.move(0);
    chassis.turnToPoint(-65, -63, 1000,{.forwards=false});
    pros::delay(1000);
    rightMotors.move(-50);
    leftMotors.move(-50);
    pros::delay(900);
    rightMotors.move(0);
    leftMotors.move(0);
 //end of right red corner
    setMogo(false);
    pros::delay(500);
    rightMotors.move(50);
    leftMotors.move(50);
    pros::delay(300);
    rightMotors.move(0);
    leftMotors.move(0);
    chassis.turnToPoint(-47.5, -48, 1000);
    chassis.moveToPoint(-47.5, -48, 1000);
    pros::delay(50);
    chassis.turnToPoint(-47.5, 0, 1500,{.forwards=false});
    chassis.moveToPoint(-47.5, 0, 5000,{.forwards=false});
    // move from right corner to mid line
     // edit these two for transition with full field
     
    chassis.turnToPoint(-48.3, 22, 1500,{.forwards=false}); //edit these two with yeah
    chassis.moveToPoint(-48.3, 22, 1500, {.forwards=false,.maxSpeed=50});
    conveyor.move(0);
    pros::delay(600);
    //edit delay and stuff once clamp
    setMogo(true);
    pros::delay(500);  
    chassis.turnToPoint(-54, 47.5, 1500);
    setIntake(11000);
    chassis.moveToPoint(-54, 47.5, 1500);
    pros::delay(1000);
    setIntake(11000);
    pros::delay(1000);
    chassis.turnToPoint(-64.5, 63, 1500,{.forwards=false});
    chassis.moveToPoint(-64.5, 63, 1500,{.forwards=false});
    conveyor.move(0);
    pros::delay(1500);

    setMogo(false);
    pros::delay(200); // WORKING, 19 POINTS
   
    rightMotors.move(50);
    leftMotors.move(50);
    pros::delay(200);
    rightMotors.move(0);
    leftMotors.move(0);
    setIntake(127);
    chassis.turnToPoint(-24, 48.5, 1000); // add to y
    chassis.moveToPoint(-24, 48.5, 3000);
   
   
    chassis.turnToPoint(39, 33, 1500,{.forwards=false});
    chassis.moveToPoint(39, 33, 4000,{.forwards=false});
    pros::delay(500);
    chassis.turnToPoint(52, 25.5, 1500,{.forwards=false});
    chassis.moveToPoint(52, 25.5, 3000,{.forwards=false, .maxSpeed=50});
    pros::delay(1000);
    setMogo(true);
    pros::delay(500);
    rightMotors.move(50);
    leftMotors.move(50);
    pros::delay(300);
    rightMotors.move(0);
    leftMotors.move(0);
    pros::delay(500);
    chassis.turnToPoint(65, 64, 1000, {.forwards=false});
    chassis.moveToPoint(65, 64, 3000,{.forwards=false});
    pros::delay(1000);
    setMogo(false);
    pros::delay(300);
    rightMotors.move(50);
    leftMotors.move(50);
    pros::delay(200);
    rightMotors.move(0);
    leftMotors.move(0);
    chassis.turnToPoint(48, 13, 1500,{.forwards=false});
    chassis.moveToPoint(48, 13, 3000,{.forwards=false});
    pros::delay(1000);
    chassis.turnToPoint(45,6,1500,{.forwards=false});
    chassis.moveToPoint(45,6,4000,{.forwards=false,.maxSpeed=70});
    pros::delay(3000);
    setMogo(true);
    pros::delay(1000);
    conveyor.move(127);
    setIntake(11000);
    pros::delay(1000);
    chassis.turnToPoint(48,-30,1500,{.forwards=false});
    chassis.moveToPoint(48,-30,2000,{.forwards=false});
    pros::delay(1000);
    chassis.turnToPoint(65,-65,1500,{.forwards=false});
    chassis.moveToPoint(65,-65,2000,{.forwards=false});






   








   

    // match auton
    /*
    }
    
    int side = 1;//1 is blue, -1 is red
    int which = -1; // 1 is side with 4 rings, -1 is side with extra mogo
    int theta = 0;
    if (which == 1){
        theta = 0;
    }
    if (which == -1){
        theta = 180;
    }
    chassis.setPose(58*side,12*which,theta);
    chassis.moveToPoint(58*side, 0,2000, {.forwards=false});
    chassis.turnToHeading(-90*side, 1000);
    pros::delay(1000);
    rightMotors.move(-30);
    leftMotors.move(-30);
    pros::delay(300);
    rightMotors.move(0);
    leftMotors.move(0);
   
    conveyor.move(127);
    pros::delay(2000);
    rightMotors.move(30);
    leftMotors.move(30);
    pros::delay(500); // remove
    rightMotors.move(0);
    leftMotors.move(0);
   
    pros::delay(300);
    chassis.turnToPoint(27*side, 23*which, 1000, {.forwards=false});  
    pros::delay(1001);
   chassis.moveToPoint(27*side, 23*which, 1500, {.forwards=false,.maxSpeed=50});  
    conveyor.move(0);
    pros::delay(2000);
    clamp.set_value(true);
    pros::delay(200);
    rightMotors.move(0);
    leftMotors.move(0);
    pros::delay(1000);
    chassis.turnToPoint(23*side, 48*which, 2000);
    fsintake.move(127);
    chassis.moveToPoint(23*side, 48*which, 3000);
    pros::delay(500);
    fsintake.move(0);
    pros::delay(200);
    conveyor.move(127);
    if (which == 1){
        chassis.turnToPoint(10*side, 45, 2000);
        chassis.moveToPoint(10*side, 45,  2000);        
        pros::delay(300);
        rightMotors.move(-50);
        leftMotors.move(-50);
        pros::delay(500);
        rightMotors.move(0);
        leftMotors.move(0);
        pros::delay(300);
    }
    //chassis.turnToPoint(13*side, 18*which, 1000);
    //chassis.moveToPoint(13*side, 18*which, 1000);
     
   */

   
}  


    


   








   

    // match auton
    /*
    int side = 1;//1 is blue, -1 is red
    int which = -1; // 1 is side with 4 rings, -1 is side with extra mogo
    int theta = 0;
    if (which == 1){
        theta = 0;
    }
    if (which == -1){
        theta = 180;
    }
    chassis.setPose(58*side,12*which,theta);
    chassis.moveToPoint(58*side, 0,2000, {.forwards=false});
    chassis.turnToHeading(-90*side, 1000);
    pros::delay(1000);
    rightMotors.move(-30);
    leftMotors.move(-30);
    pros::delay(300);
    rightMotors.move(0);
    leftMotors.move(0);
   
    conveyor.move(127);
    pros::delay(2000);
    rightMotors.move(30);
    leftMotors.move(30);
    pros::delay(500); // remove
    rightMotors.move(0);
    leftMotors.move(0);
   
    pros::delay(300);
    chassis.turnToPoint(27*side, 23*which, 1000, {.forwards=false});  
    pros::delay(1001);
   chassis.moveToPoint(27*side, 23*which, 1500, {.forwards=false,.maxSpeed=50});  
    conveyor.move(0);
    pros::delay(2000);
    clamp.set_value(true);
    pros::delay(200);
    rightMotors.move(0);
    leftMotors.move(0);
    pros::delay(1000);
    chassis.turnToPoint(23*side, 48*which, 2000);
    fsintake.move(127);
    chassis.moveToPoint(23*side, 48*which, 3000);
    pros::delay(500);
    fsintake.move(0);
    pros::delay(200);
    conveyor.move(127);
    if (which == 1){
        chassis.turnToPoint(10*side, 45, 2000);
        chassis.moveToPoint(10*side, 45,  2000);        
        pros::delay(300);
        rightMotors.move(-50);
        leftMotors.move(-50);
        pros::delay(500);
        rightMotors.move(0);
        leftMotors.move(0);
        pros::delay(300);
    }
    //chassis.turnToPoint(13*side, 18*which, 1000);
    //chassis.moveToPoint(13*side, 18*which, 1000);
     
   */


void progSkills1() {
    //set everything 0
    setIntake(0);
    setMogo(false);

    chassis.setPose(-58, 0, 90);
    //getting alliance stake
    setIntake(10000);  //INTAKING
    pros::delay(400);

    //going to inbetween mogos
    chassis.moveToPoint(-44, 0, 1000, {.forwards = true, .maxSpeed = 127}, true);
    chassis.turnToHeading(0, 1000, {}, true);
    setIntake(0);
    
    //going to first mogo
    chassis.moveToPoint(-44.5, -15.1, 1000, {.forwards = false}, false);
    pros::delay(400);
    setMogo(true);
    pros::delay(400);

    //going to 1st donut
    setIntake(10000);
   // chassis.moveToPose(-28.5, -19, 125, 1000, {.forwards = true}, true);
  //  chassis.moveToPose(-27.78, -24.8, 90, 1000, {.forwards = true}, true);
    chassis.turnToPoint(-27.78, -24.8, 1000, {.forwards = true}, true);
    chassis.moveToPoint(-27.78, -24.8, 1000, {.forwards = true}, true);

    //going to 2nd donut
    chassis.turnToPoint(-25.25, -41.49, 1000, {.forwards = true}, true);
    chassis.moveToPoint(-25.25, -41.49, 1000, {.forwards = true}, true);
    
    
    //lining to other donuts
    chassis.moveToPose(-23, -47, 180, 1000, {.forwards = true},  true);

    //going to corner area donuts
    //3rd donut
    //chassis.moveToPose(-22,-47, 270, 1000, {.forwards = true}, true );
    chassis.moveToPose(-40.5, -46.8, 270, 1000, {.forwards = true}, true);
    //4th donut
    chassis.turnToPoint(-43.09, -53, 1000, {.forwards = true}, true);
    chassis.moveToPoint(-43.09, -53, 1000, {.forwards = true}, true);

    //5th donut
    chassis.turnToPoint(-58.52, -47.56, 1000, {.forwards = true}, true);
    chassis.moveToPoint(-58.52, -47.56, 1000, {.forwards = true}, true);

    //going to corner and depositing mogo
    chassis.moveToPose(-58.2, -56, 30, 1000, {.forwards = false});
    pros::delay(300);
    setMogo(false);
    pros::delay(300);

    //set everything to 0
    setIntake(0);
    setMogo(false);
} //end of progSkills2


void test() {
    chassis.setPose(-59.03, -0, 90);
    //getting alliance stake
    setIntake(10000);  //INTAKING
    pros::delay(400);

    //going to inbetween mogos
    chassis.moveToPoint(-46, 0, 1000, {.forwards = true, .maxSpeed = 127}, true);
    chassis.turnToHeading(0, 1000, {}, true);
    setIntake(0);
    
    //going to first mogo
    chassis.moveToPoint(-47, -14, 1000, {.forwards = false}, true);
    pros::delay(100);
    setMogo(true);
}

void progSkillsSimple() {
    //set everything 0
    setIntake(0);
    setMogo(false);

    chassis.setPose(-58, 0, 90);
    //getting alliance stake
     setIntake(12000);  //INTAKING
    pros::delay(400);
    pros::delay(300);

    //going to inbetween mogos and turning
    chassis.moveToPoint(-44.5, 0, 5000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToHeading(0, 5000, {}, false);
    setIntake(0);

     //going to first mogo and clamping
    chassis.moveToPoint(-44.5, -15.1, 5000, {.forwards = false}, false);
    pros::delay(200);
    setMogo(true);
    pros::delay(200);
    chassis.moveToPoint(-44.5, -22, 5000, {.forwards = false,}, false);

    //going to 1st donut (and now technically second)
    setIntake(10000);
    chassis.moveToPoint(-44.5, -44.3, 5000, {.forwards = true, .maxSpeed = 65},false);
    pros::delay(300);
    chassis.moveToPoint(-44.5, -52.3, 5000, {.forwards = true, .maxSpeed = 65}, false); //going more down
    pros::delay(1450);
    chassis.moveToPoint(-44.5, -50.3, 5000, {.forwards = false, .maxSpeed = 65}, false);//going backwards

    //going to 2nd donut
    chassis.turnToPoint(-47.7, -50.3, 5000, {.forwards = true, .maxSpeed = 127},false); //changed to add jiggle bit
    chassis.moveToPoint(-47.7, -50.3, 5000, {.forwards = true, .maxSpeed = 65},false);
    pros::delay(200);
    chassis.moveToPoint(-49.7, -50.3, 5000, {.forwards = true, .maxSpeed = 65},false); //moving a bit more forward
    pros::delay(950);
    chassis.moveToPoint(-47.7, -50.3, 5000, {.forwards = false, .maxSpeed = 65},false); //going back to before


    //getting in corner (NEED TO FIX)
  //  chassis.swingToHeading(10, lemlib::DriveSide::LEFT, 5000, {.maxSpeed = 65},false);
    chassis.moveToPose(-47.7, -50.3,0, 2500, {.forwards = true, .maxSpeed = 65}, true );
    chassis.turnToPoint(-64.33, -65,2500, {.forwards = false, .maxSpeed = 65}, false );
    chassis.moveToPose(-64.33, -65, 30, 2500, {.forwards = false}, false);
    //chassis.moveToPose(-59, -59.665, 0, 2500, {.forwards = false}, false);//ADDED NOW NOT SURE IF GOOD
    //not sure if theta should be 0 or 180 ^^

    //dropping mogo off
    pros::delay(200);
    setMogo(false);
    pros::delay(200);
    chassis.setPose(-64.33,-65,45); //recalibrate
    pros::delay(150);
    chassis.moveToPoint(-47,-47,1000);
    
    //testing a bit of second half//

     //NEGATING INTAKE REMIND TO MAKE IT NORMAL AGAIN
    setIntake(-10000);

    //going to other mogo
    chassis.moveToPose(-42, 16, 0, 1000, {.forwards = false, .maxSpeed = 65}, false);
    chassis.moveToPose(-42, 25, 0, 1000, {.forwards = false, .maxSpeed =65}, false);
    //clamping
    setIntake(0);
   chassis.moveToPoint(-42, 27, 5000, {.forwards = false, .maxSpeed = 65}, false);
    pros::delay(200);
    setMogo(true);
    pros::delay(300);
    chassis.moveToPoint(-42, 27, 5000, {.forwards = false, .maxSpeed = 65}, false);

     //going to 1st donut (and now technically second)
    setIntake(10000);
    chassis.moveToPoint(-43, 44.3, 5000, {.forwards = true, .maxSpeed = 65},false);
    pros::delay(300);
    chassis.moveToPoint(-44, 52.3, 5000, {.forwards = true, .maxSpeed = 65}, false); //going more down
    pros::delay(1450);
    chassis.moveToPoint(-44, 50.3, 5000, {.forwards = false, .maxSpeed = 65}, false);//going backwards

    //going to 2nd donut
    chassis.turnToPoint(-44, 50.3, 5000, {.forwards = true, .maxSpeed = 127},false); //changed to add jiggle bit
    chassis.moveToPoint(-44, 50.3, 5000, {.forwards = true, .maxSpeed = 65},false);
    pros::delay(200);
    chassis.moveToPoint(-47.7, 50.3, 5000, {.forwards = true, .maxSpeed = 65},false); //moving a bit more forward
    pros::delay(950);
    chassis.moveToPoint(-47.7, 50.3, 5000, {.forwards = false, .maxSpeed = 65},false); //going back to before

    //getting in corner (NEED TO FIX)
  //  chassis.swingToHeading(10, lemlib::DriveSide::LEFT, 5000, {.maxSpeed = 65},false);
    chassis.moveToPose(-47.7, 50.3,180, 5000, {.forwards = true, .maxSpeed = 65}, true ); 
    chassis.turnToPoint(-68, 68,5000, {.forwards = false, .maxSpeed = 65}, false );
    chassis.moveToPose(-68, 68, 150, 5000, {.forwards = false}, false);
    //chassis.moveToPose(-58.25, 59.665, 180, 5000, {.forwards = false}, false);//ADDED NOW NOT SURE IF GOOD
    //not sure if theta should be 0 or 180 ^^

     //dropping mogo off
    pros::delay(200);
    setMogo(false);
    pros::delay(200);
    

     /*//moving to other mogo in the back
    chassis.moveToPose(-24.75, 35.7, 90, 5000, {.forwards = true, .maxSpeed = 85}, false);
    chassis.moveToPose(-1.07, 45.42, 90, 5000, {.forwards = true, .maxSpeed = 85}, false);
    chassis.moveToPose(32.5, 35.7, 90,5000, {.forwards = false, .maxSpeed = 85}, false); //turn around halfway
    chassis.moveToPose(50.8, 29.6, 90, 5000, {.forwards = false, .maxSpeed = 85}, false);
    //clamping
    pros::delay(200);
    setMogo(true);
    pros::delay(200);
    //going into corner
    chassis.turnToPoint(59, 53.6, 5000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(63, 65.4, 5000, {.forwards = false, .maxSpeed = 127}, false);
    */
    //opposite side thingy hopefully it works lmao


    /*
    chassis.turnToPoint(-43.413, 47.9,5000, {.forwards = true, .maxSpeed = 65}, false );
    chassis.moveToPose(-43.413, 47.9, 30, 5000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.moveToPose(-43.413, 63, 30, 5000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.moveToPose(-46, 52.3,0, 5000, {.forwards = true, .maxSpeed = 65}, true );
    chassis.turnToPoint(-64.33, 65,5000, {.forwards = false, .maxSpeed = 65}, false );
    chassis.moveToPose(-57.28, 57.69, 30, 5000, {.forwards = false}, false);
    chassis.moveToPose(-58.25, 59.665, 0, 5000, {.forwards = false}, false);
    chassis.moveToPose(53.389, 23.185,0, 5000, {.forwards = false, .maxSpeed = 65}, false );
    chassis.moveToPose(53.631, 57.14,270, 5000, {.forwards = false, .maxSpeed = 65}, false );
    chassis.moveToPose(63.257, 61.836,0, 5000, {.forwards = false, .maxSpeed = 65}, false );
    */

    //SECOND HALF
    
    /*

    //going to other mogo
    chassis.moveToPose(-44.5, 0, 0, 1000, {.forwards = true}, false);
    chassis.moveToPose(-44.5, 14.5, 0, 1000, {.forwards = false}, false);
    pros::delay(200);
    setMogo(true);
    pros::delay(200);
    chassis.moveToPoint(-44.5, -22, 5000, {.forwards = false,}, false); //moving a lil more back

    //going to 1st donut
    chassis.moveToPoint(-44.5, 44.3, 5000, {.forwards = true, .maxSpeed = 65},false);

    //going to 2nd donut
    chassis.turnToPoint(-47.7, 43, 5000, {.forwards = true, .maxSpeed = 65},false);
    chassis.moveToPoint(-47.7, 43, 5000, {.forwards = true, .maxSpeed = 65},false);

    //getting in corner
    chassis.swingToHeading(170, lemlib::DriveSide::RIGHT, 5000, {.maxSpeed = 65},false);
    //chassis.turnToPoint(-45.9, 8.2, 5000, {.forwards = true, .maxSpeed = 65}, false);
    chassis.moveToPoint(-59.5, 56,5000, {.forwards = false, .maxSpeed = 65}, false );

    //dropping mogo off
    pros::delay(300);
    setMogo(false);
    pros::delay(300);
    */

    //UNCOMMENT DURING COMP
    /*
    //moving to other mogo in the back
    chassis.moveToPose(-24.75, 35.7, 90, 5000, {.forwards = true, .maxSpeed = 65} false);
    chassis.moveToPose(32.5, 35.7, 90,5000 {.forwards = false, .maxSpeed = 65}, false); //turn around halfway
    chassis.moveToPose(50.8, 29.6, 125, 5000, {.forwards = false, .maxSpeed = 65}, false);
    //clamping
    pros::delay(200);
    setMogo(true);
    pros::delay(200);
    //going into corner
    chassis.turnToPoint(59, 53.6, 5000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(63, 65.4, 5000, {.forwards = false, .maxSpeed = 127}, false);
    */
    
    //set everything 0
    setIntake(0);
    setMogo(false);

}//end of progSkillsSimple()

void progSkillsnoLB() {
    //set everything 0
    setIntake(0);
    setMogo(false);

    chassis.setPose(-58, 0, 90);
    //getting alliance stake
     setIntake(12000);  //INTAKING
    pros::delay(400);
    pros::delay(300);

    //going to inbetween mogos and turning
    chassis.moveToPoint(-44.5, 0, 5000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToHeading(0, 5000, {}, false);
    setIntake(0);

     //going to first mogo and clamping
    chassis.moveToPoint(-44.5, -15.1, 5000, {.forwards = false}, false);
    pros::delay(200);
    setMogo(true);
    pros::delay(200);
    chassis.moveToPoint(-44.5, -22, 5000, {.forwards = false,}, false);
    setIntake(12000);

    //collect donut above
    chassis.turnToPoint(-21,-23.7, 1000, {.forwards = true, .maxSpeed= 127}, false);
    chassis.moveToPoint(-21, -23.7, 1000, {.forwards = true, .maxSpeed = 127}, false);
    pros::delay(500);
    
    //collecting mid donut
    chassis.turnToPoint(1, 1, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(1, 1, 1000, {.forwards = true, .maxSpeed = 127}, false);
    pros::delay(500);

    //move back to reposition for other donuts
    chassis.turnToPoint(-21, -23.7, 1000, {.forwards = true, . maxSpeed = 127},  false);
    chassis.moveToPoint(-21, -23.7, 1000, {.forwards = true, .maxSpeed = 127}, false);
    pros::delay(100);
    chassis.turnToPoint(-21, -48.5, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-21, -48.5, 1000, {.forwards = true, .maxSpeed = 127}, false); //donut pickup 1
    pros::delay(500);
    chassis.turnToPoint(-61, -48, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-49, -48, 1000, {.forwards = true, .maxSpeed = 127}, false);
    pros::delay(150);
    chassis.moveToPoint(-61, -48, 1000, {.forwards = true, .maxSpeed = 127}, false); //pickup 2
    pros::delay(550);
    chassis.turnToPoint(-45, -60, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-45, -60, 1000, {.forwards = true, .maxSpeed = 127}, false); //last donut of stack
    pros::delay(500);

    chassis.turnToPoint(-45, -45, 1000, {.forwards = true, .maxSpeed =127}, false);
    chassis.moveToPoint(-45, -45, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToHeading(45, 1000, {}, false);
    setIntake(0);

    chassis.turnToPoint(-68, -68, 1000, {.forwards = false, .maxSpeed = 127}, false); // dropping mogo off
    chassis.moveToPoint(-68, -68, 1000, {.forwards = false, .maxSpeed = 127}, false);
    setMogo(false);
    chassis.setPose(-69, -65, 45); //realign

    chassis.moveToPoint(-47, -47, 1000, {.forwards = true, .maxSpeed = 127}, false); //position for next mogo
    chassis.turnToPoint(-47, 24, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-47, 24, 1000, {.forwards = false, .maxSpeed = 127}, false);
    setMogo(true);

    chassis.turnToPoint(-20, 26, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-20, 26, 1000, {.forwards = true, .maxSpeed = 127}, false); //pickup 1st donut of new stack
    pros::delay(450);

    chassis.turnToPoint(-23, 49, 1000, {.forwards = true, .maxSpeed = 127}, false);

}

void progSkills2() {
    //set everything 0
    setIntake(0);
    setMogo(false);


    chassis.setPose(-58, 0, 90);
    //getting alliance stake
    setIntake(10000);  //INTAKING
    pros::delay(400);


    //going to inbetween mogos
    chassis.moveToPoint(-44, 0, 5000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToHeading(0, 5000, {}, false);
    setIntake(0);
   
    //going to first mogo
    chassis.moveToPoint(-44.5, -15.1, 5000, {.forwards = false}, false);
    pros::delay(200);
    setMogo(true);
    pros::delay(200);
    chassis.moveToPoint(-44.5, -22, 5000, {.forwards = false,}, false);
    //going to 1st donut
    setIntake(10000);
    chassis.turnToPoint(-23.2, -23.3, 5000, {.forwards = true,.maxSpeed = 65}, false);
    chassis.moveToPoint(-23.2, -23.3, 5000, {.forwards = true,.maxSpeed = 65}, false);
    
    //added going forward/backwards
    pros::delay(150);
    chassis.moveToPoint(-20.2, -23.3, 5000, {.forwards = true, .maxSpeed = 75}, false); //forward trying to intake
    pros::delay(150);
    chassis.moveToPoint(-23.2, -23.3, 5000, {.forwards = false, .maxSpeed = 65}, false); //backward
    

    //going to 2nd donut
    
    chassis.turnToPoint(-18, -49, 5000, {.forwards = true,.maxSpeed = 65}, false);
    chassis.moveToPoint(-18, -49, 5000, {.forwards = true,.maxSpeed = 65}, false);   
   
   //going to all donuts in a line
    chassis.turnToPoint(-58.2, -50.59, 5000, {.forwards = true, .maxSpeed = 65}, false);
    chassis.moveToPoint(-56.2, -50.59, 5000, {.forwards = true, .maxSpeed = 65}, false);


    //going to corner and depositing mogo
    chassis.turnToPoint(-58, -54, 1000, {.forwards = false,.maxSpeed = 65}, false);
    chassis.moveToPoint(-58, -54, 1000, {.forwards = false,.maxSpeed = 65}, false);

    
    pros::delay(300);
    setMogo(false);
    pros::delay(300);

    

    //going to other mogo
    chassis.moveToPose(-44.5, 0, 0, 1000, {.forwards = true}, false);
    chassis.moveToPose(-44.5, 14.5, 0, 1000, {.forwards = false}, false);
    pros::delay(200);
    setMogo(true);
    pros::delay(200);

    //OTHER HALF
    

    

    //set everything to 0
    setIntake(0);
    setMogo(false);
} //end of progSkills2




void redNegative() {
     //set everything 0
    setIntake(0);
    setMogo(false);
    //will try to go for 4 rings total.

    
    chassis.setPose(-62, 36.3, 270);

    //going to first mogo
    chassis.moveToPoint(-40.38, 36.35, 1000, {.forwards = false, .maxSpeed = 65}, false);
    chassis.turnToPoint(-24, 24, 1000, {.forwards = false, .maxSpeed = 65}, false);
    chassis.moveToPoint(-24, 24, 1000,  {.forwards = false, .maxSpeed = 65}, false);

    //getting mogo
    pros::delay(200);
    setMogo(true);
    pros::delay(200);

    //turning to solo stack
    chassis.turnToPoint(-24, 46, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    pros::delay(50);
    setIntake(12000);
    pros::delay(300);
    chassis.moveToPoint(-24, 46, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    pros::delay(800);
    //FROM HERE COPY PASTE FOR SOLO AWP^, not really tho cause not doable for world qual awp

    //turning to mid stack
    chassis.turnToPoint(-10, 44, 1000,{.forwards = true, .maxSpeed = 65}, false);
    chassis.moveToPoint(-10, 44, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    pros::delay(600);

    //backing up
    //setIntake(-12000);//reversing incase stuck
    chassis.moveToPoint(-23, 46, 1000,  {.forwards = false, .maxSpeed = 65}, false);
    
    //going back to mid stack for other donut
    chassis.turnToPoint(-9.3, 49.45, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    setIntake(12000);
    chassis.moveToPoint(-9.3, 49.45, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    pros::delay(600);

    //backing up again
    chassis.moveToPoint(-23, 46, 1000,  {.forwards = false, .maxSpeed = 65}, false);
    setIntake(-12000);
    //going to ladder
    chassis.moveToPoint(-23.5, 5, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    chassis.moveToPoint(-23.5, 5, 1000,  {.forwards = true, .maxSpeed = 65}, false);

    //set everything 0
    setIntake(0);
  //  setMogo(false);

}//redNegative

void progSkills3() {
    chassis.setPose(-58, 0, 90);
    setMogo(false);
    moveArmToPosition(0);
 
 
    //putting ring on stake with conveyer
    setIntake(-12000);
    pros::delay(300);
 
 
    //getting mogo
    setIntake(12000);
    chassis.moveToPoint(-46, 0, 5000, {.forwards = true}, false);
    chassis.turnToHeading(0, 5000, {}, false);
    chassis.moveToPoint(-47, -12, 5000, {.forwards = false}, false);
    setMogo(true);
    pros::delay(300);
 
 
    //get closest donut
    chassis.turnToPoint(-24,-24, 1000, {.forwards = true, .maxSpeed= 127}, false);
    chassis.moveToPoint(-24, -24, 5000, {.forwards = true}, false);
    pros::delay(300);
 
 
    //set lady brown get red donut near positive blue corner
    moveArmToPosition(1);
    chassis.turnToPoint(24,-47, 1000, {.forwards = true, .maxSpeed= 127}, false);
    chassis.moveToPoint(24, -47, 5000, {.forwards = true}, false);
    pros::delay(300);
 
 
    //put ring on stake
    chassis.turnToPoint(0,-47, 1000, {.forwards = false, .maxSpeed= 127}, false);
    chassis.moveToPoint(0, -47, 5000, {.forwards = false}, false);
    pros::delay(300);
    chassis.turnToHeading(180, 5000, {}, false);
    chassis.moveToPoint(0, -57, 5000, {.forwards = true}, false);
    pros::delay(300);
    moveArmToPosition(2);
    pros::delay(500);
    moveArmToPosition(0);
 
 
    //move right for nearest ring, get next 3 rings, set down mogo
    chassis.moveToPoint(0, -47, 5000, {.forwards = false}, false);
    chassis.turnToHeading(270, 5000, {}, false);
    chassis.moveToPoint(-59, -47, 5000, {.forwards = true}, false);
    pros::delay(300);
    chassis.turnToPoint(-47,-59, 1000, {.forwards = true, .maxSpeed= 127}, false);
    chassis.moveToPoint(-47, -59, 5000, {.forwards = true}, false);
    pros::delay(300);
    chassis.turnToPoint(-58,-59, 1000, {.forwards = false, .maxSpeed= 127}, false);
    setMogo(false);
 
 
    //go to other side of field, get clamp
    chassis.turnToPoint(47,-47, 1000, {.forwards = true, .maxSpeed= 127}, false);
    moveArmToPosition(1);
    chassis.moveToPoint(47, -47, 5000, {.forwards = true}, false);
    pros::delay(300);
    setIntake(0);
    chassis.turnToPoint(62,-21, 1000, {.forwards = false, .maxSpeed= 127}, false);
    chassis.moveToPoint(62, -21, 5000, {.forwards = false}, false);
    setMogo(true);
    pros::delay(300);
    chassis.turnToHeading(180, 5000, {}, false);
 
 
    //extend doinker, sweep rings out of corner
    setDoinker(true);
    chassis.moveToPoint(63, -50, 5000, {.forwards = true}, false);
    chassis.turnToHeading(270, 5000, {}, false);
    setDoinker(false);
    setIntake(12000);
 
 
    //release mogo, get another mogo
    chassis.turnToPoint(61,-58, 1000, {.forwards = false, .maxSpeed= 127}, false);
    chassis.moveToPoint(61, -58, 5000, {.forwards = false}, false);
    setMogo(false);
    chassis.turnToPoint(47,-47, 1000, {.forwards = true, .maxSpeed= 127}, false);
    chassis.moveToPoint(47, -47, 5000, {.forwards = true}, false);
    pros::delay(300);
    chassis.turnToHeading(180, 5000, {}, false);
    chassis.moveToPoint(47, -12, 5000, {.forwards = false}, false);
    setMogo(true);
    pros::delay(300);
 
 
    //put ring on stake
    chassis.moveToPoint(47, 0, 5000, {.forwards = false}, false);
    chassis.turnToHeading(90, 5000, {}, false);
    chassis.moveToPoint(58, 0, 5000, {.forwards = true}, false);
    pros::delay(300);
    moveArmToPosition(2);
    pros::delay(300);
    moveArmToPosition(0);
 
 
    //go down field then across field to get rings
    chassis.moveToPoint(46, 0, 5000, {.forwards = false}, false);
    chassis.turnToPoint(24,-24, 1000, {.forwards = true, .maxSpeed= 127}, false);
    chassis.moveToPoint(24, -24, 5000, {.forwards = true}, false);
    pros::delay(300);
    chassis.turnToPoint(-47,47, 1000, {.forwards = true, .maxSpeed= 127}, false);
    chassis.moveToPoint(-47, 47, 5000, {.forwards = true}, false);
    pros::delay(300);
    chassis.turnToHeading(270, 5000, {}, false);
    chassis.moveToPoint(-58, 47, 5000, {.forwards = true}, false);
    pros::delay(300);
    chassis.turnToPoint(-47,59, 1000, {.forwards = true, .maxSpeed= 127}, false);
    chassis.moveToPoint(-47, 59, 5000, {.forwards = true}, false);
    pros::delay(300);
    chassis.turnToHeading(315, 5000, {}, false);
    chassis.moveToPoint(-58, 59, 5000, {.forwards = false}, false);
    pros::delay(300);
    setMogo(false);
 
 
    //more ladybrown bs
    moveArmToPosition(1);
    chassis.turnToPoint(-24,47, 1000, {.forwards = true, .maxSpeed= 127}, false);
    chassis.moveToPoint(-24, 47, 5000, {.forwards = true}, false);
    pros::delay(300);
    chassis.turnToPoint(-40,32, 1000, {.forwards = false, .maxSpeed= 127}, false);
    chassis.moveToPoint(-40, 32, 5000, {.forwards = false}, false);
    pros::delay(300);
    chassis.turnToPoint(0,52, 1000, {.forwards = false, .maxSpeed= 127}, false);
    chassis.moveToPoint(0, 52, 5000, {.forwards = false}, false);
    pros::delay(300);
    chassis.turnToHeading(0, 5000, {}, false);
    chassis.moveToPoint(0, 57, 5000, {.forwards = false}, false);
    moveArmToPosition(2);
    pros::delay(300);
    moveArmToPosition(0);
 
 
    //idk anymore man make it stop
    chassis.turnToPoint(23,47, 1000, {.forwards = true, .maxSpeed= 127}, false);
    chassis.moveToPoint(23, 47, 5000, {.forwards = true}, false);
    pros::delay(300);
    chassis.turnToPoint(23,23, 1000, {.forwards = true, .maxSpeed= 127}, false);
    chassis.moveToPoint(23, 23, 5000, {.forwards = true}, false);
    pros::delay(300);
    chassis.turnToPoint(47,47, 1000, {.forwards = true, .maxSpeed= 127}, false);
    chassis.moveToPoint(47, 47, 5000, {.forwards = true}, false);
    pros::delay(300);
    chassis.turnToPoint(59,47, 1000, {.forwards = true, .maxSpeed= 127}, false);
    chassis.moveToPoint(59, 47, 5000, {.forwards = true}, false);
    pros::delay(300);
    chassis.turnToPoint(47,59, 1000, {.forwards = true, .maxSpeed= 127}, false);
    chassis.moveToPoint(47, 59, 5000, {.forwards = true}, false);
    pros::delay(300);
    chassis.turnToPoint(32,59, 1000, {.forwards = false, .maxSpeed= 127}, false);
    chassis.moveToPoint(32, 59, 5000, {.forwards = false}, false);
    pros::delay(300);
    setDoinker(true);
    chassis.turnToPoint(54,59, 1000, {.forwards = true, .maxSpeed= 127}, false);
    chassis.moveToPoint(54, 59, 5000, {.forwards = true}, false);
    pros::delay(300);
    chassis.turnToHeading(225, 5000, {}, false);
    setMogo(false);
 }
 
 
 
 
 

void redPositive() {
    //set everything 0
    setIntake(0);
    setMogo(false);

    //going to first mogo
    chassis.setPose(-61.5, -11.5, 270);
    chassis.moveToPoint(-41.38, -11.5, 1000,{.forwards = false, .maxSpeed = 65}, false  ); //midpoint
    chassis.turnToPoint(-24.25, -22.87, 1000, {.forwards = false, .maxSpeed = 65}, false);
    chassis.moveToPoint(-24.25, -22.87, 1000, {.forwards = false, .maxSpeed = 65}, false);//actually touched mogo here
    //clamping
     pros::delay(200);
    setMogo(true);
    pros::delay(200);

    //turning to single stack
    chassis.turnToPoint(-24, -43.04, 1000, {.forwards = false, .maxSpeed = 65}, false);
    pros::delay(100);
    setIntake(12000);
    pros::delay(100);
    chassis.moveToPoint(-24, -43.04, 1000, {.forwards = true, .maxSpeed = 65},false);
    pros::delay(650);

    //FROM HERE ON EITHER GO STRAIGHT TO LADDER CODE *OR* positive and mogo rush

    //straight to ladder..


    //putting mogo near positive.

    //dropping mogo off
    chassis.turnToPoint(-41.3, -52, 1000, {.forwards = false, .maxSpeed = 65}, true);
    chassis.moveToPoint(-41.3, -52, 1000, {.forwards = false, .maxSpeed = 65}, true);
    setMogo(false);
    pros::delay(200);
    chassis.moveToPoint(-41.3,-38,1000,{.forwards = false, .maxSpeed = 65}, true);
    //aligning with mogo
    chassis.turnToPoint(-24.5, -46, 1000, {.forwards = false, .maxSpeed = 65}, true);

    chassis.moveToPoint(-24.5, -46, 1000, {.forwards = false, .maxSpeed = 65}, true);

    //bum rushing mid mogo
    chassis.turnToPoint(-10, -46, 1000, {.forwards = false, .maxSpeed = 65}, false);
    chassis.moveToPoint(-10, -46, 1000, {.forwards = false, .maxSpeed = 65}, false);

    //clamping 2nd mogo
     pros::delay(200);
    setMogo(true);
    pros::delay(200);

    //going to ladder
    chassis.moveToPoint(-21.7, -6.9, 1000, {.forwards = true, .maxSpeed = 127}, false);

    //set everything 0
    setIntake(0);
   // setMogo(false);
}

void blueNegative() {
     //set everything 0
    setIntake(0);
    setMogo(false);
    //will try to go for 4 rings total.

    
    chassis.setPose(62, 36.3, 90);

    //going to first mogo
    chassis.moveToPoint(40.38, 36.35, 1000, {.forwards = false, .maxSpeed = 65}, false);
    chassis.turnToPoint(24, 24, 1000, {.forwards = false, .maxSpeed = 65}, false);
    chassis.moveToPoint(24, 24, 1000,  {.forwards = false, .maxSpeed = 65}, false);

    //getting mogo
    pros::delay(200);
    setMogo(true);
    pros::delay(200);

    //turning to solo stack
    chassis.turnToPoint(24, 46, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    pros::delay(50);
    setIntake(12000);
    pros::delay(300);
    chassis.moveToPoint(24, 46, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    pros::delay(800);
    //FROM HERE COPY PASTE FOR SOLO AWP^, not really tho cause not doable for world qual awp

    //turning to mid stack
    chassis.turnToPoint(10, 44, 1000,{.forwards = true, .maxSpeed = 65}, false);
    chassis.moveToPoint(10, 44, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    pros::delay(600);

    //backing up
    //setIntake(-12000);//reversing incase stuck
    chassis.moveToPoint(24, 46, 1000,  {.forwards = false, .maxSpeed = 65}, false);
    
    //going back to mid stack for other donut
    chassis.turnToPoint(10, 49.45, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    setIntake(12000);
    chassis.moveToPoint(10, 49.45, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    pros::delay(600);

    //backing up again
    chassis.moveToPoint(23, 46, 1000,  {.forwards = false, .maxSpeed = 65}, false);
    //setIntake(-12000);
    //going to ladder
    chassis.moveToPoint(23.5, 5, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    chassis.moveToPoint(23.5, 5, 1000,  {.forwards = true, .maxSpeed = 65}, false);

    //set everything 0
    setIntake(0);
  //  setMogo(false);

}//redNegative

void bluePositive() {
    //set everything 0
    setIntake(0);
    setMogo(false);

    //going to first mogo
    chassis.setPose(61.5, -11.5, 90);
    chassis.moveToPoint(41.38, -11.5, 1000,{.forwards = false, .maxSpeed = 65}, false  ); //midpoint
    chassis.turnToPoint(24.25, -22.87, 1000, {.forwards = false, .maxSpeed = 65}, false);
    chassis.moveToPoint(24.25, -22.87, 1000, {.forwards = false, .maxSpeed = 65}, false);//actually touched mogo here
    //clamping
     pros::delay(200);
    setMogo(true);
    pros::delay(200);

    //turning to single stack
    chassis.turnToPoint(24, -43.04, 1000, {.forwards = false, .maxSpeed = 65}, false);
    pros::delay(100);
    setIntake(12000);
    pros::delay(100);
    chassis.moveToPoint(24, -43.04, 1000, {.forwards = true, .maxSpeed = 65},false);
    pros::delay(900);

    //FROM HERE ON EITHER GO STRAIGHT TO LADDER CODE *OR* positive and mogo rush

    //straight to ladder..


    //putting mogo near positive.

    //dropping mogo off
    chassis.turnToPoint(41.3, -52, 1000, {.forwards = false, .maxSpeed = 127}, true);
    chassis.moveToPoint(41.3, -52, 1000, {.forwards = false, .maxSpeed = 127}, true);
    setMogo(false);

    pros::delay(200);
    chassis.moveToPoint(41.3,-38,1000,{.forwards = false, .maxSpeed = 65}, true);
    //aligning with mogo
    chassis.turnToPoint(24.5, -46, 1000, {.forwards = false, .maxSpeed = 65}, true);
    chassis.moveToPoint(24.5, -46, 1000, {.forwards = false, .maxSpeed = 127}, true);

    //bum rushing mid mogo
    chassis.turnToPoint(11, -47.5, 1000, {.forwards = true, .maxSpeed = 65}, false);
    chassis.moveToPoint(11, -47.5, 1000, {.forwards = true, .maxSpeed = 65}, false);
    pros::delay(500);
    setDoinker(true);
    setIntake(0);

    /*clamping 2nd mogo
     pros::delay(200);
    setMogo(true);
    pros::delay(200);
*/
    chassis.moveToPoint(30,-46,1000,{.forwards = false, .maxSpeed = 65}, false);
    setDoinker(false);
    pros::delay(100);

    chassis.turnToPoint(67,-67,1000, {.forwards = true, .maxSpeed =127}, false);
    setDoinker(true);
    chassis.moveToPoint(57,-57, 1000,{.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(69,-60,1000,{.forwards=true, .maxSpeed = 127}, false);
    chassis.turnToPoint(60,-69, 1000, {.forwards= true, .maxSpeed = 127}, false);
    chassis.turnToPoint(69,-60,1000,{.forwards=true, .maxSpeed = 127}, false);
    chassis.turnToPoint(60,-69, 1000, {.forwards= true, .maxSpeed = 127}, false);
    //going to ladder
    //chassis.moveToPoint(21.7, -6.9, 1000, {.forwards = true, .maxSpeed = 127}, false);

    //set everything 0
//    setIntake(0);
   // setMogo(false);
}

void blueNegativeElim() {
     //set everything 0
    setIntake(0);
    setMogo(false);
    //will try to go for 4 rings total.

    
    chassis.setPose(62, 36.3, 90);

    //going to first mogo
    chassis.moveToPoint(40.38, 36.35, 1000, {.forwards = false, .maxSpeed = 65}, false);
    chassis.turnToPoint(24, 24, 1000, {.forwards = false, .maxSpeed = 65}, false);
    chassis.moveToPoint(24, 24, 1000,  {.forwards = false, .maxSpeed = 65}, false);

    //getting mogo
    pros::delay(200);
    setMogo(true);
    pros::delay(200);

    //turning to solo stack
    chassis.turnToPoint(24, 46, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    pros::delay(50);
    setIntake(12000);
    pros::delay(300);
    chassis.moveToPoint(24, 46, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    pros::delay(800);
    //FROM HERE COPY PASTE FOR SOLO AWP^, not really tho cause not doable for world qual awp

    //turning to mid stack
    chassis.turnToPoint(10, 44, 1000,{.forwards = true, .maxSpeed = 65}, false);
    chassis.moveToPoint(10, 44, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    pros::delay(600);

    //backing up
    //setIntake(-12000);//reversing incase stuck
    chassis.moveToPoint(24, 46, 1000,  {.forwards = false, .maxSpeed = 65}, false);
    
    //going back to mid stack for other donut
    chassis.turnToPoint(10, 49.45, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    setIntake(12000);
    chassis.moveToPoint(10, 49.45, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    pros::delay(600);

    //backing up again
    chassis.moveToPoint(23, 46, 1000,  {.forwards = false, .maxSpeed = 65}, false);
    
    //setIntake(-12000);
    /*going to ladder
    chassis.moveToPoint(23.5, 5, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    chassis.moveToPoint(23.5, 5, 1000,  {.forwards = true, .maxSpeed = 65}, false);
*/

    //set everything 0
    setIntake(0);
    setDoinker(true);

    chassis.turnToPoint(69, 69, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(57, 57, 1000, {.forwards = true, .maxSpeed = 70}, false);
    chassis.turnToPoint( 63, 69, 1000, {.forwards = true, .maxSpeed = 80}, false);
    chassis.turnToPoint(70, 60, 1000, {.forwards = true, .maxSpeed = 80}, false);
    chassis.turnToPoint( 63, 69, 1000, {.forwards = true, .maxSpeed = 80}, false);
    chassis.turnToPoint(70, 60, 1000, {.forwards = true, .maxSpeed = 80}, false);
    chassis.turnToPoint( 63, 69, 1000, {.forwards = true, .maxSpeed = 80}, false);
    chassis.turnToPoint(70, 60, 1000, {.forwards = true, .maxSpeed = 80}, false);
    chassis.turnToPoint( 63, 69, 1000, {.forwards = true, .maxSpeed = 80}, false);
    chassis.turnToPoint(70, 60, 1000, {.forwards = true, .maxSpeed = 80}, false);
  //  setMogo(false);

}
void bluePositiveElim() {
    //set everything 0
    setIntake(0);
    setMogo(false);

    //going to first mogo
    chassis.setPose(61.5, -11.5, 90);
    chassis.moveToPoint(41.38, -11.5, 1000,{.forwards = false, .maxSpeed = 75}, false  ); //midpoint
    chassis.turnToPoint(24.25, -22.87, 1000, {.forwards = false, .maxSpeed = 75}, false);
    chassis.moveToPoint(24.25, -22.87, 1000, {.forwards = false, .maxSpeed = 75}, false);//actually touched mogo here
    //clamping
     pros::delay(200);
    setMogo(true);
    pros::delay(200);

    //turning to single stack
    chassis.turnToPoint(24, -43.04, 1000, {.forwards = false, .maxSpeed = 75}, false);
    pros::delay(300);
    setIntake(12000);
    pros::delay(700);
    chassis.moveToPoint(24, -43.04, 1000, {.forwards = true, .maxSpeed = 75},false);
    pros::delay(700);

    //FROM HERE ON EITHER GO STRAIGHT TO LADDER CODE *OR* positive and mogo rush

    //straight to ladder..


    //putting mogo near positive.

    //dropping mogo off
    chassis.turnToPoint(41.3, -32, 1000, {.forwards = false, .maxSpeed = 127}, true);
    chassis.moveToPoint(41.3, -32, 1000, {.forwards = false, .maxSpeed = 127}, true);
    setMogo(false);

    pros::delay(200);
    chassis.moveToPoint(41.3,-38,1000,{.forwards = false, .maxSpeed = 75}, true);
    //aligning with mogo
    chassis.turnToPoint(24.5, -46, 1000, {.forwards = false, .maxSpeed = 70}, true);
    chassis.moveToPoint(24.5, -46, 1000, {.forwards = false, .maxSpeed = 127}, true);

    //bum rushing mid mogo
    chassis.turnToPoint(11, -42.5, 1000, {.forwards = true, .maxSpeed = 70}, false);
    chassis.moveToPoint(11, -42.5, 1000, {.forwards = true, .maxSpeed = 70}, false);
    pros::delay(350);
    setDoinker(true);
    setIntake(0);

    /*clamping 2nd mogo
     pros::delay(200);
    setMogo(true);
    pros::delay(200);
*/
    chassis.moveToPoint(30,-46,1000,{.forwards = false, .maxSpeed = 85}, false);
    setDoinker(false);
    pros::delay(100);

    chassis.turnToPoint(67,-67,1000, {.forwards = true, .maxSpeed =127}, false);
    setDoinker(true);
    chassis.moveToPoint(54.5,-57.5, 1000,{.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(69,-60,1000,{.forwards=true, .maxSpeed = 127}, false);
    chassis.turnToPoint(60,-69, 1000, {.forwards= true, .maxSpeed = 127}, false);
    chassis.turnToPoint(69,-60,1000,{.forwards=true, .maxSpeed = 127}, false);
    chassis.turnToPoint(60,-69, 1000, {.forwards= true, .maxSpeed = 127}, false);
    chassis.turnToPoint(69,-60,1000,{.forwards=true, .maxSpeed = 127}, false);
    chassis.turnToPoint(60,-69, 1000, {.forwards= true, .maxSpeed = 127}, false);
    chassis.turnToPoint(69,-60,1000,{.forwards=true, .maxSpeed = 127}, false);
    chassis.turnToPoint(60,-69, 1000, {.forwards= true, .maxSpeed = 127}, false);
    //going to ladder
    //chassis.moveToPoint(21.7, -6.9, 1000, {.forwards = true, .maxSpeed = 127}, false);

    //set everything 0
//    setIntake(0);
   // setMogo(false);
}

void redNegativeElim() {
     //set everything 0
    setIntake(0);
    setMogo(false);
    //will try to go for 4 rings total.

    
    chassis.setPose(-62, 36.3, 270);

    //going to first mogo
    chassis.moveToPoint(-40.38, 36.35, 1000, {.forwards = false, .maxSpeed = 65}, false);
    chassis.turnToPoint(-24, 24, 1000, {.forwards = false, .maxSpeed = 65}, false);
    chassis.moveToPoint(-24, 24, 1000,  {.forwards = false, .maxSpeed = 65}, false);

    //getting mogo
    pros::delay(200);
    setMogo(true);
    pros::delay(200);

    //turning to solo stack
    chassis.turnToPoint(-24, 46, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    pros::delay(50);
    setIntake(12000);
    pros::delay(300);
    chassis.moveToPoint(-24, 46, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    pros::delay(800);
    //FROM HERE COPY PASTE FOR SOLO AWP^, not really tho cause not doable for world qual awp

    //turning to mid stack
    chassis.turnToPoint(-10, 44, 1000,{.forwards = true, .maxSpeed = 65}, false);
    chassis.moveToPoint(-10, 44, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    pros::delay(600);

    //backing up
    //setIntake(-12000);//reversing incase stuck
    chassis.moveToPoint(-24, 46, 1000,  {.forwards = false, .maxSpeed = 65}, false);
    
    //going back to mid stack for other donut
    chassis.turnToPoint(-10, 49.45, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    setIntake(12000);
    chassis.moveToPoint(-10, 49.45, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    pros::delay(600);

    //backing up again
    chassis.moveToPoint(-23, 46, 1000,  {.forwards = false, .maxSpeed = 65}, false);
    
    //setIntake(-12000);
    /*going to ladder
    chassis.moveToPoint(23.5, 5, 1000,  {.forwards = true, .maxSpeed = 65}, false);
    chassis.moveToPoint(23.5, 5, 1000,  {.forwards = true, .maxSpeed = 65}, false);
*/

    //set everything 0
    setIntake(0);
    setDoinker(true);

    chassis.turnToPoint(-69, 69, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-57, 57, 1000, {.forwards = true, .maxSpeed = 70}, false);
    chassis.turnToPoint( -63, 69, 1000, {.forwards = true, .maxSpeed = 80}, false);
    chassis.turnToPoint(-70, 60, 1000, {.forwards = true, .maxSpeed = 80}, false);
    chassis.turnToPoint( -63, 69, 1000, {.forwards = true, .maxSpeed = 80}, false);
    chassis.turnToPoint(-70, 60, 1000, {.forwards = true, .maxSpeed = 80}, false);
    chassis.turnToPoint( -63, 69, 1000, {.forwards = true, .maxSpeed = 80}, false);
    chassis.turnToPoint(-70, 60, 1000, {.forwards = true, .maxSpeed = 80}, false);
    chassis.turnToPoint( -63, 69, 1000, {.forwards = true, .maxSpeed = 80}, false);
    chassis.turnToPoint(-70, 60, 1000, {.forwards = true, .maxSpeed = 80}, false);
  //  setMogo(false);

}
void redPositiveElim() {
    //set everything 0
    setIntake(0);
    setMogo(false);

    //going to first mogo
    chassis.setPose(-61.5, -11.5, 270);
    chassis.moveToPoint(-41.38, -11.5, 1000,{.forwards = false, .maxSpeed = 70}, false  ); //midpoint
    chassis.turnToPoint(-24.25, -22.87, 1000, {.forwards = false, .maxSpeed = 70}, false);
    chassis.moveToPoint(-24.25, -22.87, 1000, {.forwards = false, .maxSpeed = 70}, false);//actually touched mogo here
    //clamping
     pros::delay(200);
    setMogo(true);
    pros::delay(200);

    //turning to single stack
    chassis.turnToPoint(-24, -43.04, 1000, {.forwards = false, .maxSpeed = 70}, false);
    pros::delay(100);
    setIntake(12000);
    pros::delay(500);
    chassis.moveToPoint(-24, -43.04, 1000, {.forwards = true, .maxSpeed = 70},false);
    pros::delay(700);

    //FROM HERE ON EITHER GO STRAIGHT TO LADDER CODE *OR* positive and mogo rush

    //straight to ladder..


    //putting mogo near positive.

    //dropping mogo off
    chassis.turnToPoint(-41.3, -52, 1000, {.forwards = false, .maxSpeed = 127}, true);
    chassis.moveToPoint(-41.3, -52, 1000, {.forwards = false, .maxSpeed = 127}, true);
    setMogo(false);

    pros::delay(200);
    chassis.moveToPoint(-41.3,-38,1000,{.forwards = false, .maxSpeed = 70}, true);
    //aligning with mogo
    chassis.turnToPoint(-24.5, -49, 1000, {.forwards = false, .maxSpeed = 65}, true);
    chassis.moveToPoint(-24.5, -49, 1000, {.forwards = false, .maxSpeed = 127}, true);

    //bum rushing mid mogo
    chassis.turnToPoint(-11, -55, 1000, {.forwards = true, .maxSpeed = 70}, false);
    chassis.moveToPoint(-11, -55, 1000, {.forwards = true, .maxSpeed = 70}, false);
    pros::delay(500);
    setDoinker(true);
    setIntake(0);

    /*clamping 2nd mogo
     pros::delay(200);
    setMogo(true);
    pros::delay(200);
*/
    chassis.moveToPoint(-30,-55,1000,{.forwards = false, .maxSpeed = 75}, false);
    setDoinker(false);
    pros::delay(100);

    chassis.turnToPoint(-67,-67,1000, {.forwards = true, .maxSpeed =127}, false);
    setDoinker(true);
    chassis.moveToPoint(-57,-57, 1000,{.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-69,-60,1000,{.forwards=true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-60,-69, 1000, {.forwards= true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-69,-60,1000,{.forwards=true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-60,-69, 1000, {.forwards= true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-69,-60,1000,{.forwards=true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-60,-69, 1000, {.forwards= true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-69,-60,1000,{.forwards=true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-60,-69, 1000, {.forwards= true, .maxSpeed = 127}, false);
    //going to ladder
    //chassis.moveToPoint(21.7, -6.9, 1000, {.forwards = true, .maxSpeed = 127}, false);

    //set everything 0
//    setIntake(0);
   // setMogo(false);
}


//STATES AUTONS MADE BY DNYANESH ON 2/19/25
void statesRedPositive() {
    chassis.setPose(-48.039, -31.745, 120);
    //set everything 0
    setMogo(false);
    setDoinker(false);
    setIntake(10000);

    //going to first stack and holding red ring
    chassis.moveToPoint(-26.51, -43.72, 1000, {.forwards = true, .maxSpeed = 65}, false);
    pros::delay(250);
    //setIntake(0);

    //going to mid mogo rush
    chassis.moveToPoint(-23.11, -46.7, 1000, {.forwards = true, .maxSpeed = 65}, false); //move a bit to get donut in
    setIntake(0);
    intake.move_voltage(-12000);  //BECAUSE OF STUPID AHH BLUE

    chassis.moveToPoint(-14.4, -46.8, 1000, {.forwards = true, .maxSpeed = 65}, false);
    //setIntake(0);
    chassis.moveToPoint(-13.4, -46.8, 1000, {.forwards = true, .maxSpeed = 65}, false);
    setDoinker(true);
    pros::delay(200);

    //goingbackwards with mogo
    chassis.moveToPoint(-28, -44.36, 1000, {.forwards = false, .maxSpeed = 65}, false);
    //chassis.moveToPose(-28, -44.36, 90, 1000, {.forwards = false, .maxSpeed = 40}, false);
    chassis.turnToHeading(96, 1000, {.maxSpeed = 50}, false);
    setDoinker(false);
    pros::delay(200);
    
    //going to mogo and scoring
    chassis.turnToHeading(270, 1000, {.maxSpeed = 65}, false);
    chassis.moveToPoint(-24.5, -42.36, 1000, {.forwards = false, .maxSpeed = 50}, false); //going to mogo
    pros::delay(200);
    setMogo(true);
    pros::delay(100);
    setIntake(10000);
    pros::delay(500);

    //moving forward to align with next mogo
    chassis.moveToPoint(-23.6, -42.36, 1000, {.forwards = true, .maxSpeed = 65}, false); //going to mogo

    //letting go of old mogo
    setIntake(0);
    setMogo(false);
    pros::delay(100);

    //going to other mogo and clamping
    chassis.turnToPoint(-23.6, -23.1, 1000, {.forwards = false, .maxSpeed = 65}, false);
    chassis.moveToPoint(-23.6, -23.1, 1000, {.forwards = false, .maxSpeed = 50}, false);
    pros::delay(200);
    setMogo(true);
    pros::delay(100);
    setIntake(10000);

    //going to preload and scoring
    chassis.moveToPoint(-62.5, -23.9, 1000, {.forwards = true, .maxSpeed = 65}, false);
    
    //elims (go to corner to get ready to clear)

    


}

void statesRedPositiveFAST() {
    chassis.setPose(-48.039, -31.745, 120);
    //set everything 0
    setMogo(false);
    setDoinker(false);
  //  setIntake(10000);
    intake.move_voltage(12000);

    //going to first stack and holding red ring
    chassis.moveToPoint(-26.51, -43.72, 1000, {.forwards = true}, true);
    pros::delay(250);
    //setIntake(0);

    //going to mid mogo rush
    chassis.moveToPoint(-23.11, -46.7, 1000, {.forwards = true}, true); //move a bit to get donut in
   // setIntake(0);
   // intake.move_voltage(-12000);  //BECAUSE OF STUPID AHH BLUE

    chassis.moveToPoint(-14.4, -46.8, 1000, {.forwards = true}, true);
    //setIntake(0);
    chassis.moveToPoint(-13.4, -46.8, 1000, {.forwards = true}, true);
    setDoinker(true);
    pros::delay(200);

    //goingbackwards with mogo
    chassis.moveToPoint(-28, -44.36, 1000, {.forwards = false, .maxSpeed = 65}, false);
    //chassis.moveToPose(-28, -44.36, 90, 1000, {.forwards = false, .maxSpeed = 40}, false);
    chassis.turnToHeading(96, 1000, {.maxSpeed = 50}, false);
    setDoinker(false);
    pros::delay(200);
    
    //going to mogo and scoring
    chassis.turnToHeading(270, 1000, {.maxSpeed = 65}, false);
    chassis.moveToPoint(-24.5, -42.36, 1000, {.forwards = false, .maxSpeed = 50}, false); //going to mogo
    pros::delay(200);
    setMogo(true);
    pros::delay(100);
   // setIntake(10000);
   conveyor.move_voltage(10000); //scoring 1st ring
    pros::delay(300);

    //moving forward to align with next mogo
    chassis.moveToPoint(-23.6, -42.36, 1000, {.forwards = true, .maxSpeed = 65}, false); //going to mogo

    //letting go of old mogo
    setIntake(0);

    setMogo(false);
    pros::delay(100);

    //going to other mogo and clamping
    chassis.turnToPoint(-23.6, -23.1, 1000, {.forwards = false, .maxSpeed = 65}, false);
    chassis.moveToPoint(-23.6, -23.1, 1000, {.forwards = false, .maxSpeed = 50}, false);
    pros::delay(200);
    setMogo(true);
    pros::delay(100);
    setIntake(10000);

    //going to preload and scoring
   // chassis.moveToPoint(-62.5, -23.9, 1000, {.forwards = true, .maxSpeed = 65}, false);
    
    //elims (go to corner to get ready to clear)

    
    

}


void statesBluePositive() {

}

void autonomous() {
    //setting lb to hold..?
    ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

   // statesRedPositive();

    statesRedPositiveFAST();

    
  //while (true) {
  //  colorSortRed();  //takes out reds
   // colorSortBlue(); //takes out blues

    
   //paste whatever auton in here
   //blueNegative();
   
 // }
  
    // moveAuton();
//  progSkillsSimple(); //tries to get one ring on 2 mogos and 3 mogos into corners
  // progSkills2(); //new ROUTE and will probably  use
    // test(); //just going to mogo

    //blueNegative();
    //redNegative();
    //bluePositive();
    //redPositive();
    //blueNegativeElim();
   // bluePositiveElim();
    //redNegativeElim();
    //rePositiveElim();
	/*
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
	*/

}





/**
 * Runs in driver control
 */
void opcontrol() {
    while (true) {
        drive();

        setMogoSolenoids();

        setIntakeMotors();

        setLadyBrown();

        //moveArmToPosition();

        setDoinkerSolenoids();

     //   setDoinker();
    }
    // controller
    // loop to continuously update motors
   
}