#include "main.h"

//helper function
void setIntake(int power) { //intake power (I need to figure out how to reverse and forward)
    //Need to only do one motor and let it reverse ofc
    //figuring out like the triggers 
    //in globals: true is negative and false is positive
    intake.move_voltage(power);
    belt.move_voltage(power);
}

//driver controller functions
void setIntakeMotors() {
    //bottom trigger intakes and top trigger outtakes
    //link belt to the same thigies
    
    int intakePower = 11000*((controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) - (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) );  
     //maybe fix l1/l2 later..
     //also this code acts like 127* ( 1 - 0) = 127 OR 127*(0-1) = -127 (max speds)
    
    
    // Clamp intakePower to the range of -12000 to 12000
   // intakePower = std::max(-12000, std::min(12000, intakePower));


    setIntake(intakePower);
    /*
    if(controller.get.digital(pros::E_CONTROLLER_DIGITAL_L2)) 
        intakePower = 11000; //negative max sped
    else if (controller.get.digital(pros::E_CONTROLLER_DIGITAL_L1)) 
        intakePower = 11000; //positive max sped
    setIntake(intakePower);

    INEFFICIENT CODE
    */


} //only one motor wil be set
