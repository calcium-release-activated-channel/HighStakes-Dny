#include "main.h"

bool extendPiston = false; // Declare outside the function for persistent state

void setMogo(bool extend) {
    pneumaticTwo.set_value(extend);
    pneumaticOne.set_value(extend);
}

void setMogoSolenoids() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        extendPiston = !extendPiston; // Toggle the state
        setMogo(extendPiston); // Apply the toggled state to pneumatics
        pros::delay(300); // Delay to prevent multiple toggles from a single press
    }
}



