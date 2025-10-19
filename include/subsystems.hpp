#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
    inline pros::Motor intake(6);
    inline pros::Motor hood(3);
    inline pros::Motor combine(4);
    inline pros::MotorGroup intake_combine({-6, 4}); // Intake and Combine motors
    inline pros::adi::Pneumatics block_collector('A', false); // Pneumatic for the block collector
