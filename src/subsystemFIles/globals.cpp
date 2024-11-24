#include "main.h"
#include "pros/motors.hpp"
#include "pros/adi.hpp"

//setting up motors-

/*
PLEASE KEEP IN MIND FOR THE FUTURE THAT I NEED TO CHANGE THE VALUES FOR BOTH CARTRIDGES AND  direction
Also I need to learn pnuematics...

*/

pros::Motor intake(-20, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts); 
//(+- Port , Gearset (rpm), encoder units (a.k.a counting stuff))
// intake.set_reversed(true);

pros::Motor belt(5, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);


//left drive
pros::Motor driveLeftFront(-18, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
pros::Motor driveLeftBack(-19, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
pros::Motor driveLeftBackTop(9, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);

//right drive
pros::Motor driveRightFront(11, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
pros::Motor driveRightBack(15, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts); //,BEFORE  pros::E_MOTOR_ENCODER_COUNTS
pros::Motor driveRightBackTop(-13, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);

//controller
pros::Controller controller(pros::E_CONTROLLER_MASTER); 

//pnuematics
//NEED TO CHECK LATER I HAVE NO CLUE!!!
// pros::ADIDigitalOut pnuematicOne('A'); //A is like the port it should be plugged into...? idk
// pros::ADIDigitalOut pnuematicTwo('B');                              //B is like the port thing it should be plugged into? idk adding anoteher motor

pros::adi::DigitalOut pneumaticOne('A');
pros::adi::DigitalOut pneumaticTwo('A');
 
//auton selector button?
//pros::adi::DigitalOut button('F');

//sensors 
// pros::ADIDigitalIn autonSelector(autonSelectorPort); AUTON SELECTOR
// pros::Imu imuObj(imuPort);   IMU FOR AUTON
//MAY LATER IMPLEMENT TRACKING WHEELS

pros::IMU inertial_sensor(17); //change port later
