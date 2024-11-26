#include "main.h"
//#include "intake.hpp"
//#include "mogo.hpp"
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
//lemLib
// motor groups
pros::MotorGroup driveL({-18, -19, 9}, pros::MotorGears::blue);
pros::MotorGroup driveR({14,15,-13}, pros::MotorGears::blue);

// PID
lemlib::Drivetrain autonDrive{&driveL, &driveR, 9.5, lemlib::Omniwheel::NEW_325, 4, 300};
lemlib::ControllerSettings linearController{10, 0, 3, 3, 1, 100, 3, 500, 20};
lemlib::ControllerSettings angularController{2, 0, 10, 3, 1, 100, 3, 500, 0};
lemlib::OdomSensors sensors{nullptr, nullptr, nullptr, nullptr, &inertial_sensor};
lemlib::Chassis autonChassis(autonDrive, linearController, angularController, sensors);


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
   // autonChassis.calibrate();

    
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Nanesh wuz here :3");

    // MOTOR STUFF    
    driveLeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);  // motor do whatever (keep movingg), hold- keep it, brake- like hold but won't counteract motion
    driveLeftBackTop.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);  //ORIGINALLY ALL WERE COAST, MAY CHANGE BACK BECAUSE AUTON...
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
/* Key:
 * "*" == Working
 * "-" == Programmed but untested
 * " " == Not programmed
 */
const std::vector<std::string> autonModes = {
    " Default    ",
    " RedFar     ",
    " RedClose   ",
    " BlueFar    ",
    " BlueClose  "};
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
    autonChassis.setPose(-57.524, 44.169, 300); //initally backwarsd
    //going to mogo and intaking preload
    autonChassis.moveToPoint(-10.502, 15.7, 2000, {.forwards=false});
    setMogo(true);
    pros::delay(200);
    setIntake(10000);
    pros::delay(600); 
    
     //going back to og mogo spot
    autonChassis.moveToPoint(-23.936, 23.697, 2000);
    //turning to stack
    autonChassis.turnToPoint(-4.104, 43.209, 2000);
    autonChassis.moveToPoint(-4.104, 43.209, 2000);
   //going backwards
    autonChassis.moveToPoint(-17.859, 33.933, 2000,{.forwards=false});
    autonChassis.turnToPoint(-4.104, 50.566, 2000);
    //going to stack again
     autonChassis.moveToPoint(-4.104, 50.566, 2000);
     //going backwards
     autonChassis.moveToPoint(-19.458, 29.134, 2000, {.forwards=false});
     //going to second stack
     autonChassis.turnToPoint(-23.936, 47.367, 2000);
    autonChassis.moveToPoint(-23.936, 47.367, 2000);
    //going back to ladder
   // autonChassis.moveToPoint(-23.936, 1.305, 2000, {.forwards=false});

    setIntake(0);
    setMogo(false); //remove later TESTING
}

void redFarOld2() {
    setMogo(false);
    //going to mogo and intaking preload
    translate(-1550,100);
    pros::delay(200);
    setMogo(true);
    pros::delay(400);
    setIntake(10000);

    pros::delay(600);
    translate(490,70);
    rotate(-115,40); // turn to first stack
    translate2(745,70); //was 90
    pros::delay(200);
    translate2(-340,90); //was 100
    pros::delay(100);
    translate2(347,70); //was 357
    translate2(3,127);
    pros::delay(300);
    translate2(-845,100);
    rotate(14,40); // turn to second stack
    translate2(1002,70); //was 1012
    pros::delay(200);
    translate2(-340,100);
    pros::delay(100);
    translate2(354,70);
    translate2(2,127);
    pros::delay(500);
    // returning to initial mogo area
    translate(-870,100);
    pros::delay(200);
    rotate(33,40); // turn to single stack
    pros::delay(200);
    translate(580,90); //was 585
    translate(3,127); //quick forward to intake
    pros::delay(1500);
    setIntake(0);
    //translate(-500,127); //ladder
}

void redFarOld() {
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

    setIntake(0);
    
}
void redFarBKUP() {
    setMogo(false);
    //going to mogo and intaking preload
    translate(-1480,100);
    pros::delay(200);
    setMogo(true);
    pros::delay(500);
    translate(270,70); // go to initial mogo pos.
    pros::delay(450);
    setIntake(10000);
    pros::delay(600);
    rotate(-70,40);
    pros::delay(200);
    translate2(190,85); // go to solo stack
    pros::delay(1050);
    translate2(-20,100);
    pros::delay(100);
    translate2(25,100);
    pros::delay(100);
    translate2(-15,100);
    pros::delay(100);
    translate2(10,100);
    pros::delay(300);
    rotate(-70,40); //mid turn
    pros::delay(300);
    translate2(350,50);
    pros::delay(300);
    translate2(-180,60);
    pros::delay(200);
    translate(85,127);
    pros::delay(300);
    translate(-335,100); //reversing 4 ladder
    pros::delay(400);
    rotate(-32,40);
    pros::delay(200);
    translate(520,100); //move toward ladder
}
void redFar2()  {
 setMogo(false);
    //going to mogo and intaking preload
    translate(-1540,80); //REMINDER 11/26 slowed velocity 100 ---> 80
    //SWITCH BACK IF MOGO ISN'T CLAMPNIG ^
    //ALSO POSSIBLY SWITCH TO TRANSLATE2 (no correcting, however this may mess up clamp more for making path better)
    pros::delay(200);
    setMogo(true);
    pros::delay(500);
    translate(300,60);

    // go to initial mogo pos. and intake 
    pros::delay(450);
    setIntake(10000);
    pros::delay(600);

    //rotating to solo stack
    rotate(-60,40);
    pros::delay(200);
    translate2(240,85); // go to solo stack
    pros::delay(1050);
    translate2(-20,110);
    pros::delay(150);
    translate2(22,110); //change?
    pros::delay(150);
    translate2(-12,100);
    pros::delay(150);
    translate2(10,100);
    pros::delay(330);

    //turning to mid stack
    rotate(-73,40); //mid turn
    pros::delay(250);
    translate2(350,50); //could be too much, change if getting blue rings
    pros::delay(250);
    translate2(-180,60);    
    pros::delay(200);
    translate(80,127); //changed 80 ---> 85 //may change if cross + 
    pros::delay(350);
    translate(-335,100); 
    pros::delay(400);
    rotate(-40,40);
    pros::delay(200);
    translate(540,100);
    
}

void redFar2Elim()  {
 setMogo(false);
//if any changes made to redFar2(), most likely change this one...
 
    //going to mogo and intaking preload
    translate(-1540,80); //REMINDER 11/26 slowed velocity 100 ---> 80
    //SWITCH BACK IF MO GO ISN'T CLAMPNIG ^
    //ALSO POSSIBLY SWITCH TO TRANSLATE2 (no correcting--> this may mess up clamp more for making path better)
    pros::delay(200);
    setMogo(true);
    pros::delay(500);
    translate(300,60);

    // go to initial mogo pos. and intake 
    pros::delay(450);
    setIntake(10000);
    pros::delay(600);

    //rotating to solo stack
    rotate(-60,40);
    pros::delay(200);
    translate2(240,85); // go to solo stack
    pros::delay(1050);
    translate2(-20,110);
    pros::delay(150);
    translate2(22,110); //change?
    pros::delay(150);
    translate2(-12,100);
    pros::delay(150);
    translate2(10,100);
    pros::delay(330);

    //turning to mid stack
    rotate(-73,40); //mid turn
    pros::delay(250);
    translate2(350,50); //could be too much, change if getting blue rings
    pros::delay(250);
    translate2(-180,60);    
    pros::delay(200);
    translate(80,127); //changed 80 ---> 85 //may change if cross + 
    pros::delay(350);
    
    //going backwards and then to stack other side of stack
    translate(-250,100); //reversing back
    pros::delay(250);
    rotate(20, 83);//turning to the other side of the stack
    pros::delay(250);
    translate2(350,50); //initial
    pros::delay(250);
    translate2(-180,60); //back up
    pros::delay(200);
    translate(83,127); //change 85 to 80
    pros::delay(350);
    
}

void redClose() {
    //get mogo
    translate(-700, 40); //may be too far
    pros::delay(300);
    setMogo(true);
    pros::delay(200);
    //intaking preload after turning
    rotate(-90,40);
    pros::delay(200);
    setIntake(11000);
    //dropping mogo 
    pros::delay(200);
    setMogo(false);
    //knocking off top ring and PARTIALLY getting the ring intakes
    translate(520,90);
    pros::delay(1200);
    setIntake(0);
    //turning to mid mogo
    pros::delay(200);
    rotate(-90,40);
    pros::delay(200);
    translate(-200,60); //adjust depending on crossing or NOT toucing mogo
    //clamping and scoring
    setMogo(true);
    pros::delay(200);
    translate(250,80); //getting away from mid linew
    pros::delay(200);
    setIntake(11000); //scoring

    //MAY ADD GOING TO LADDER...


    
}

void redCloseNEW() {
    //get mogo
    translate(-500, 100); //may be too far
    pros::delay(300);
    rotate(-90,40);
    pros::delay(200);
    translate(-100,30);
    pros::delay(400);
    setIntake(11000);
    pros::delay(200);
    setIntake(0);
    pros::delay(100);
    translate(500,90);
    pros::delay(100);
    rotate(-135,40);
    pros::delay(100);
    translate(520,50);
    pros::delay(300);
    setMogo(true);
    pros::delay(200);
    rotate(-135,40);
    pros::delay(200);
    setIntake(11000);
    pros::delay(100);
    translate(440,90);
    pros::delay(900);
    setIntake(0);
    pros::delay(100);
    translate(-200,80);
    pros::delay(250);
    setIntake(11000);
    pros::delay(700);
    rotate(180,40);
    pros::delay(150);
    setMogo(false);
    pros::delay(100);
    translate(600,100);
    
}

void blueFar() {
    setMogo(false);
    //going to mogo and intaking preload
    translate(-1540,100);
    pros::delay(200);
    setMogo(true);
    pros::delay(500);
    translate(300,70); // go to initial mogo pos.
    pros::delay(450);
    setIntake(10000);
    pros::delay(600);
    rotate(60,40);
    pros::delay(200);
    translate2(240,85); // go to solo stack
    pros::delay(1050);
    translate2(-20,110);
    pros::delay(150);
    translate2(22,110); //change?
    pros::delay(150);
    translate2(-12,100);
    pros::delay(150);
    translate2(10,100);
    pros::delay(300);
    rotate(73,40); //mid turn
    pros::delay(250);
    translate2(350,50);
    pros::delay(250);
    translate2(-180,60);
    pros::delay(200);
    translate(80,127); //change 85 to 80
    pros::delay(350);
    translate(-335,100); //reversing 4 ladder
    pros::delay(400);
    rotate(40,40);
    pros::delay(200);
    translate(540,100);
    
}
void blueFarElim() {
    setMogo(false);
    //going to mogo and intaking preload
    translate(-1540,100);
    pros::delay(200);
    setMogo(true);
    pros::delay(500);
    translate(300,70); // go to initial mogo pos.
    pros::delay(450);
    setIntake(10000);
    pros::delay(600);
    rotate(60,40);
    pros::delay(200);
    translate2(240,85); // go to solo stack
    pros::delay(1050);
    translate2(-20,110);
    pros::delay(150);
    translate2(22,110); //change?
    pros::delay(150);
    translate2(-12,100);
    pros::delay(150);
    translate2(10,100);
    pros::delay(300);

    //turn to mid donuts
    rotate(73,40); //mid turn
    pros::delay(250);
    translate2(350,50); //initial
    pros::delay(250);
    translate2(-180,60); //back up
    pros::delay(200);
    translate(83,127); //change 85 to 80
    pros::delay(350);

    //possibly do front/back motion again? or too slow
    /*
    translate2(-180,60); //back up
    pros::delay(200);
    translate(80,127); //change 85 to 80
    pros::delay(350);
    */

    //going backwards and then to stack other side of stack
    translate(-250,100); //reversing back
    pros::delay(250);
    rotate(20, 83);//turning to the other side of the stack
    pros::delay(250);
    translate2(350,50); //initial
    pros::delay(250);
    translate2(-180,60); //back up
    pros::delay(200);
    translate(83,127); //change 85 to 80
    pros::delay(350);
}


void blueClose() {
    translate(-500, 100); //may be too far
    pros::delay(300);
    rotate(90,40);
    pros::delay(200);
    translate(-100,30);
    pros::delay(400);
    setIntake(11000);
    pros::delay(200);
    setIntake(0);
    pros::delay(100);
    translate(500,90);
    pros::delay(100);
    rotate(135,40);
    pros::delay(100);
    translate(520,50);
    pros::delay(300);
    setMogo(true);
    pros::delay(200);
    rotate(135,40);
    pros::delay(200);
    setIntake(11000);
    pros::delay(100);
    translate(440,90);
    pros::delay(900);
    setIntake(0);
    pros::delay(100);
    translate(-200,80);
    pros::delay(250);
    setIntake(11000);
    pros::delay(700);
    rotate(-180,40);
    pros::delay(150);
    setMogo(false);
    pros::delay(100);
    translate(600,100);
    
}




void noAuton() {
    //nothing
}

/*
void blueLeftCorner() {

}
*/

void autonomous () {
 //   redFarBKUP();
// redFar2Elim();
 redFar2(); //EXACT opposite of blueFAR
 //blueFarElim();   //for elims tries to go for 4 rings
 //   blueFar(); //well tuned


 //blueClose(); //wip probably never
/*
   switch (autMode) {
        case 0:
            redFarBKUP();
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
*/
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
    //setting to brake so joseph liek
    /*
    driveLeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);  // motor do whatever (keep movingg), hold- keep it, brake- like hold but won't counteract motion
    driveLeftBackTop.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);  //ORIGINALLY ALL WERE COAST, MAY CHANGE BACK BECAUSE AUTON... WHOOPS
    driveLeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    driveRightBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    driveRightFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    driveRightBackTop.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    */

    while (true) {
        
        //some code to control drive-
        setDriveMotors();

    //some code to control intake/belt-
        setIntakeMotors();  

        //controlling mogo
        setMogoSolenoids(); 

        
        pros::delay(20);                               // Run for 20 ms then update
    }
}
