#pragma once

extern bool rake_state;
extern bool angle_state;
extern bool ball_state;
extern bool descore_state;

////////////////////Pneumatics Auton///////////////////////


//
void rake_open();
void rake_close();

void angle_open();
void angle_close();

void ball_close();
void ball_open();

void descore_open();
void descore_close();

//////////////////////Pneumatic Control/////////////////
void pneumatic_control();