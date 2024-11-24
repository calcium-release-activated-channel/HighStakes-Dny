#include "main.h"
//#include "intake.hpp"
//#include "mogo.hpp"
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    }
    else {
        pros::lcd::clear_line(2);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Nanesh wuz here :3");

    // MOTOR STUFF
    driveLeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);  // motor do whatever (keep movingg), hold- keep it, brake- like hold but won't counteract motion
    driveLeftBackTop.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    driveLeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    driveRightBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    driveRightFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    driveRightBackTop.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);  // MAY CHANGE

    // HOW DO PNUEMATICS?!?!?

    
    // gyro/IMU thingy
    inertial_sensor.reset();          // Reset the inertial sensor
    pros::delay(2000);

     // needs 2 seconds to calibrate sensor.
    // gyros help to somewhat like correct urself
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */ 
void disabled() {
    // kill task
    // delay
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */

//auton selector here or aboce comp initalize
const std::vector<std::string> autonModes = {
    "* Default    ",
    "* (R) redFar",
    "* RedClose   ",
    "* BlueFar    ",
    "* BlueClose  "};
int autMode = 0;
uint8_t autonSelectorPort = 'B';
pros::adi::DigitalIn autonSelector(autonSelectorPort);

void competition_initialize() {
     pros::delay(100);
    controller.set_text(0,0, autonModes[autMode]);
    while (true) {
        if (autonSelector.get_new_press()) {
            controller.rumble(".");
            autMode = autMode < (int)autonModes.size() - 1 ? autMode + 1 : 0;
            controller.clear_line(0);
            pros::delay(50);
            controller.set_text(0, 0, autonModes[autMode]);
        }
        pros::delay(20);
    }
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


// void autonomous() {}    GETTING RID OF AUTONTOMIUOS HERE
//auton selector 


//testing VV
void redLeftCorner() {
   //translate (distance, voltage) straight line
   //rotate (degrees, voltage) 
    
    translate(500,100);
    rotate(90, 40);

    translate(-500,100);
    rotate(-90,40);



}

void redFar() {
    setMogo(false);
    //going to mogo and intaking preload
    translate(-1550,100);
    setMogo(true);
    pros::delay(200);
    setIntake(10000);

    pros::delay(600);

    //going to stack and intaking preload
    translate(300,100);
    rotate(-90,40); //turning to donuts near midline
    translate(950,100); //going to midline donuts
    pros::delay(200);
    rotate(20,40); // adjust to line up with donuts
    translate(300,100); //pickup donuts
    pros::delay(200);
    translate(-250,100);
    pros::delay(200);
    rotate(5,40);
    translate(350,100);
    pros::delay(200);
    translate(-250,100);
    pros::delay(200);
    translate(200,100); //midlinedonut pickup end
    pros::delay(200);
    rotate(-25,40); //prepare to reverse
    translate(-425,100); //reverse
    pros::delay(200);
    rotate(75,40); // turn to line up with last donut
    translate(600,100); // picking up last donut

    
}

void redClose() {
    //first line
    translate(1500, 40);

    //to goal

    /*
    rotate(60, 20);
    translate(500,20);
    setMogo(true);

    //scoring pre load
    setIntake(100);
    */
    //going for donut stack



    
}

void blueFar() {
    
    
}

void blueClose() {
    
    
}

void noAuton() {
    //nothing
}

/*
void blueLeftCorner() {

}
*/

void autonomous () {
   // redFar();

   switch (autMode) {
        case 0:
            redFar();
            break;
        case 1:
            redClose();
            break;
        case 2:
            blueFar();
            break;
        case 3:
            blueClose();
            break;
        default:
            noAuton();
            break;
    }

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */



void opcontrol() {
    //run once at the start

    while (true) {\
        
        //some code to control drive-
        setDriveMotors();

    //some code to control intake/belt-
        setIntakeMotors();  

        //controlling mogo
        setMogoSolenoids(); 

        
        pros::delay(20);                               // Run for 20 ms then update
    }
}
