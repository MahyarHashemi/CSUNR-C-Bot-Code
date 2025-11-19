#include "subsystems.hpp"
#include "main.h"

//////////////////////Outtake Auton Code///////////////////

//
void outtake_zero(){
    outtake.tare_position();
}

void outtake_set_pos(double pos, int speed){
    outtake.move_absolute(pos, speed);
}

void outtake_speed(int speed){
    outtake.move(speed);
}



/////////////////////Outtake Control///////////////////
void outtake_control(){
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        outtake.move(127);
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        outtake.move(-127);
    }
    else {
        outtake.move(0);
    }
}