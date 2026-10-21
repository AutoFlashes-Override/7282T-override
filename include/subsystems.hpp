#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

//setup motors for lift
inline pros::Motor lift1(-10); 
inline pros::Motor lift2(19);

inline void lifts(int input) {
  lift1.move(input);
  lift2.move(input);
}

inline ez::PID liftPID{0.45, 0, 0, 0, "Lift"};

//set up the claw
inline ez::Piston claw('B');

//setup motors for toggle mech
inline pros::MotorGroup togglemech({-21, 20});

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
