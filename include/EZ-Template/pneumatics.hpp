#pragma once

extern bool rake_state;
extern bool angle_state;

////////////////////Pneumatics Auton///////////////////////


//
void rake_open();
void rake_close();

void angle_open();
void angle_close();

void ball_close();
void ball_open();

//////////////////////Pneumatic Control/////////////////
void pneumatic_control();