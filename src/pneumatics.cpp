#include "subsystems.hpp"
#include "main.h"

extern bool rake_state = false;
extern bool angle_state = false;
extern bool ball_state = true;
extern bool descore_state = false;

////////////////////Pneumatics Auton///////////////////////


//
void rake_open(){
    rake.set(true);
}
void rake_close(){
    rake.set(false);
}

void angle_open(){
    angle_changer.set(true);
}
void angle_close(){
    angle_changer.set(false);
}

void ball_open(){
    ball_stop.set(false);
}
void ball_close(){
    ball_stop.set(true);
}

void descore_open(){
    descore.set(true);
}
void descore_close(){
    descore.set(false);
}

//////////////////////Pneumatic Control/////////////////
void pneumatic_control(){
    // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
    //     angle_state = !angle_state;
    // }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
        rake_state = !rake_state;
    }
    // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
    //     ball_state = !ball_state;
    // }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
        descore_state = true;
    }
    else{
        descore_state = false;
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        ball_state = false;
    }
    else{
        ball_state = true;
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        angle_state = true;
    }
    else{
        angle_state = false;
    }

    if (rake_state){
        rake.set(true);
    }
    else{
        rake.set(false);
    }

    if (angle_state){
        angle_changer.set(true);
    }
    else{
        angle_changer.set(false);
    }

    if (ball_state){
        ball_stop.set(true);
    }
    else{
        ball_stop.set(false);
    }

    if (descore_state){
        descore.set(true);
    }
    else{
        descore.set(false);
    }
    
    // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
    //     ball_stop.set(false);
    // }
    // else{
    //     ball_stop.set(true);
    // }
}

