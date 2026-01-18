#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor bottom_intake(20);
inline pros::MotorGroup middle_intake({-16, 17});
inline pros::MotorGroup outtake({-9, 10});

inline ez::Piston rake('A');

inline ez::Piston angle_changer('B');

inline ez::Piston ball_stop('C');

inline ez::Piston descore('D');

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');