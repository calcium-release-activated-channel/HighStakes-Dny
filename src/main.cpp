#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/adi.hpp"
#include <cstddef>


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
//CHANGE LATER
pros::MotorGroup leftMotors({-5, -10, 9},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({18, 14, -11}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(10);




/*
// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);
*/

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
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
                            nullptr, // horizontal tracking wheel
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
pros::Motor intake(17);
pros::Motor conveyor(16);


//pnuematics (doinker/mogo)
pros::adi::Pneumatics mogo('A', false);
pros::adi::Pneumatics doinker('B', false);

//ladyBrown
//pros::Motor ladyBrown1(-16); //left
pros::Motor ladyBrown2(19); //right



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
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        extendPiston = !extendPiston;  // Toggle the state
        setMogo(extendPiston);         // Apply the toggled state to pneumatics
        pros::delay(300);              // Delay to prevent multiple toggles from a single press
    }
    pros::delay(10);
}


/*
void setMogo() {
    bool pistonToggle = false;
    if (controller.get_digital(DIGITAL_L1)) {
        if (pistonToggle == false) {
            mogo.extend();
            pros::delay(500);
            pistonToggle = true;
        } else {
            mogo.retract();
            pros::delay(500);
            pistonToggle = false;
        }
    }
pros::delay(10);

}
*/

void setIntake(int power) {  // intake power   //call this during auton.
    
    intake.move_voltage(power);
    conveyor.move_voltage(power);
}

// driver controller functions
void setIntakeMotors() { //call this during opcontrol

    // bottom trigger intakes and top trigger outtakes
    // link belt to the same thigies

    int intakePower = 11000 * ((controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) - (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)));
   

    setIntake(intakePower);
   
}  

void setLadyPower(int power) {  // intake power   //call this during auton.
    
   // intake.move_voltage(power);
   // conveyor.move_voltage(power);
   ladyBrown2.move_voltage(power);
}

// driver controller functions
void setLadyBrown() { //call this during opcontrol

    // bottom trigger intakes and top trigger outtakes
    // link belt to the same thigies

   // int ladyPower = 11000 * ((controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) - (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)));
    int ladyPower = 11000 * ((controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) - (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)));

    setLadyPower(ladyPower);
   
}  



/*
void setDoinker() {
 bool pistonToggle = false;
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        if (pistonToggle == false) {
            doinker.extend();
            pros::delay(500);
            pistonToggle = true;
        } else {
            doinker.retract();
            pros::delay(500);
            pistonToggle = false;
        }
    
    }
    pros::delay(10);

}
*/

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    //set motors to coast
    
    //MOTOR STUFF
	
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
            pros::delay(50);
        }
    });
}

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

void blueNegative() {

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



void redNegative() {

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

   


void autonomous() {
    moveAuton();
    //blueNegative();
    //redNegative();
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

        setDoinkerSolenoids();

     //   setDoinker();
    }
    // controller
    // loop to continuously update motors
   
}