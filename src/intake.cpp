#include "subsystems.hpp"
#include "main.h"

//////////////////////Intake Auton Code///////////////////

//
void intake_zero(){
    bottom_intake.tare_position();
    middle_intake.tare_position();
}

void intake_set_pos(double pos, int speed){
    bottom_intake.move_absolute(pos, speed);
    middle_intake.move_absolute(pos, speed);
}

void intake_speed(int speed){
    bottom_intake.move(speed);
    middle_intake.move(speed);
}



/////////////////////Intake Control///////////////////
void intake_control(){
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        bottom_intake.move(127);
        middle_intake.move(127);
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        //Was 127 
        bottom_intake.move(-63);
        middle_intake.move(-127);
    }
    else {
        bottom_intake.move(0);
        middle_intake.move(0);
    }
}