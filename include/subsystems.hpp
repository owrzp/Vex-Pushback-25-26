#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
    inline pros::Motor intake(6);
    inline pros::Motor combine(4);
    inline pros::Motor hood(3);
    inline pros::adi::Pneumatics block_collector('A', false); // Pneumatic for the block collector
    inline pros::adi::Pneumatics descore_mech('B', false); // Pneumatic for the mobile goal lift
    inline pros::Distance front_distance(7);
    inline pros::Distance right_distance(8);

// Distance Sensor Abstraction
