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

//chassis motors:
inline pros::MotorGroup leftside({-14, -15, -16});
inline pros::MotorGroup rightside({11, 12, 13});
inline pros::Motor left1(-14); 
inline pros::Motor left2(-15); 
inline pros::Motor left3(-16); 
inline pros::Motor right1(11); 
inline pros::Motor right2(12); 
inline pros::Motor right3(13); 
inline ez::PID liftPID{1.0, 0, 0, 0, "Lift"};

//set up the distance sensor
inline pros::Distance Dist(9);
//set up the claw
inline ez::Piston claw('B');

//setup motors for toggle mech
inline pros::MotorGroup togglemech({-21, 20});
inline pros::Motor toggle1(-21); 
inline pros::Motor toggle2(20);

inline void Toggle(int input) {
  togglemech.move(input);
 }

 inline void lift_auto(double target) {
  liftPID.target_set(target);
  // Track the exit condition using only Lift 1
  while (liftPID.exit_condition({lift1}, true) == ez::RUNNING) {
  // Compute PID power based on the position of lift 1
  double output = liftPID.compute(lift1.get_position());
   // Check if the target is BELOW the current position (moving downward)
    if (target < lift1.get_position()) {
      // Scale down the output power. 0.5 cuts the downward speed in half.
      output = output * 0.20; 
    }
  // Send the power to both motors to lift it to the target positon
  lifts(output);
  pros::delay(ez::util::DELAY_TIME);
  }
}
// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
