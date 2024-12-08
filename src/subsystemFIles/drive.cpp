#include "main.h"

//pros::ADIGyro gyro('C', 1)  //USING THE IMU FIX PORT LATER 
//FIX LATER/// WHAT THE HELL IS OUR GRYO SCALE? gyro(port, scale)
//scale it up so for the gyro thingy it returns ur angle * 10, some gryos are weird..
// pros::Imu inertial_sensor(1); //CHANGE THE PORT VALUE


//using those functions

//Helper functions (SO WE DONT GOTTA COPY AND PASTE ALL THE TIME)
void setDriveArcade(int left, int right) {
    int leftPwr = left + right;
    int rightPwr = left - right;
    driveLeftBack.move(leftPwr);
    driveLeftFront.move(leftPwr);
    driveLeftBackTop.move(leftPwr);
    driveRightBack.move(rightPwr);
    driveRightFront.move(rightPwr);
    driveRightBackTop.move(rightPwr);
}
void setDrive(int left, int right) {
    /*
    //left side
    driveLeftFront = left;
    driveLeftBack = left;
    driveLeftBackTop = left;

    //right side
    driveRightFront = right;
    driveRightBack = right;
    driveRightBackTop = right;
    */


        //left side
        driveLeftFront.move(left);      
        driveLeftBack.move(left);       
        driveLeftBackTop.move(left);    

        //right side
        driveRightFront.move(right);    
        driveRightBack.move(right);     
        driveRightBackTop.move(right);  
}
//VVV being used in auton
void resetDriveEncoders() {
    driveLeftBack.tare_position();
    driveLeftFront.tare_position();
    driveRightBack.tare_position();
    driveRightFront.tare_position();
}

void setDriveMotors() {
   int leftJoyStick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
   int rightJoyStick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
   //analong for joysticks and digital for buttons
   //voltage to set motors is -127 to 127
   //and inputs from joystick is also -127 to 127

   //making a controller deadzone (within this area nothing happens) --> ask driver later for input
   if (abs(leftJoyStick) < 5) {
        leftJoyStick = 0;
   }

   if (abs(rightJoyStick) < 5) {
        rightJoyStick = 0;
   }

   setDrive(leftJoyStick, rightJoyStick); //defining function

   

   
   /*
    //setDrive(0,0);//left sidedriveLeftFront = 0; driveLeftBack = 0;driveLeftBackTop = 0;//right sidedriveRightFront = 0;driveRightBack = 0;driveRightBackTop = 0;
    */
}

void setDriveMotorsArcade() {
    int leftJoyStick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightJoyStick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    // analong for joysticks and digital for buttons
    // voltage to set motors is -127 to 127
    // and inputs from joystick is also -127 to 127
        

    // making a controller deadzone (within this area nothing happens) --> ask driver later for input
    if (abs(leftJoyStick) < 5) {
        leftJoyStick = 0;
    }

    if (abs(rightJoyStick) < 5) {
        rightJoyStick = 0;
    }

    setDriveArcade(leftJoyStick, rightJoyStick);  // defining function

    /*
     //setDrive(0,0);//left sidedriveLeftFront = 0; driveLeftBack = 0;driveLeftBackTop = 0;//right sidedriveRightFront = 0;driveRightBack = 0;driveRightBackTop = 0;
     */
}

//averages everything
double averageDriveEncoderValue() {
    return (fabs(driveLeftFront.get_position()) + fabs(driveLeftBack.get_position()) + 
    fabs(driveRightFront.get_position()) + fabs(driveRightBack.get_position())) / 4;
}

//autonomous
void translate(int units, int voltage) {
    //defines a direction depending on units provided
    int direction = abs(units) / units; //either -1 or 1
    
    //reset motor encoder- reset to 0 and then go more
    resetDriveEncoders();
    inertial_sensor.tare_yaw();
   // inertial_sensor.reset();

    //drive forward until units are reached
    while(averageDriveEncoderValue() < abs(units)) {  //may review
        int correction = inertial_sensor.get_yaw() * 0.5; // Scale down correction factor
        setDrive(voltage * direction + correction, voltage * direction - correction);
        pros::delay(10);  // Small delay to avoid overloading CPU
        /*
        setDrive(voltage * direction + inertial_sensor.get_yaw(), voltage * direction - inertial_sensor.get_yaw()); //before was gyro.get_value()
        //using gyro to like adjust in case we hit something.... I could possibly divide if overcorrecting..
        //furthermore imu gyro values may be outputted ifferent...
        pros::delay(10);
        */
    }
    
    //brief brake
    setDrive(-0.5 * direction,-0.5 * direction); //may change multiplier depending on units
    pros::delay(20); // (this can vary depending on heavy our bot is..)
    //set drive back to nuetral
    setDrive(0,0);

} //end of translate
//
void translate2(int units, int voltage) {
    //defines a direction depending on units provided
    int direction = abs(units) / units; //either -1 or 1
    
    //reset motor encoder- reset to 0 and then go more
    resetDriveEncoders();
    inertial_sensor.tare_yaw();
   // inertial_sensor.reset();

    //drive forward until units are reached
    while(averageDriveEncoderValue() < abs(units)) {  //may review
        int correction = inertial_sensor.get_yaw() * 0.5; // Scale down correction factor
        setDrive(voltage * direction, voltage * direction);
        pros::delay(10);  // Small delay to avoid overloading CPU
        /*
        setDrive(voltage * direction + inertial_sensor.get_yaw(), voltage * direction - inertial_sensor.get_yaw()); //before was gyro.get_value()
        //using gyro to like adjust in case we hit something.... I could possibly divide if overcorrecting..
        //furthermore imu gyro values may be outputted ifferent...
        pros::delay(10);
        */
    }
    
    //brief brake
    setDrive(-0.5 * direction,-0.5 * direction); //may change multiplier depending on units
    pros::delay(20); // (this can vary depending on heavy our bot is..)
    //set drive back to nuetral
    setDrive(0,0);

} //end of translate2

void rotate(int degrees, int voltage) {
    //define direction, based on units provided (either +  or -)
    int direction =  abs(degrees) / degrees;
            //like a unit circle i think where positive is left and negative is right
    //resetting the IMU gyro
    inertial_sensor.tare_yaw();
    //turn until units reached
    while (fabs(inertial_sensor.get_yaw()) < abs(degrees - 5)) { //-5 so we can not overshoot as much
        setDrive(-voltage * direction, voltage * direction);
        pros::delay(10);
    }
    setDrive(0,0);
    //correcting overshoot (same thing but alot slower) 
    pros::delay(100); //may change  
    if (fabs(inertial_sensor.get_yaw()) > abs(degrees)) {
        while (fabs(inertial_sensor.get_yaw()) > abs(degrees)) {
        setDrive(0.5 * voltage * direction, 0.5 * -voltage * direction); //slower
        pros::delay(10);
    }
    } else if(fabs(inertial_sensor.get_yaw()) < abs(degrees)) { //undershoot
        while (fabs(inertial_sensor.get_yaw()) < abs(degrees)) { //may need to change < to >
        setDrive(0.5 * -voltage * direction, 0.5 * voltage * direction); //slower
        pros::delay(10);
    }
    }

    //brief brake? 
    //reset drive to 0
    setDrive(0,0);
}