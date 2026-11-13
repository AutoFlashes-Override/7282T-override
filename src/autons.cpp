#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 80; //used to be 110
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(16.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.5, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);
  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are
  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}


//left side auton 
void leftsideauton(){
//FLIP TOGGLE
  claw.set(false); 
  toggle1.move_relative(-2000, 200);  //move 2000, velocity 2000
  toggle2.move_relative(-2000, 200); 
  pros::delay(450); //wait for toggles to finish turning
//SCORE PRELOAD
  lift_auto(400); //lift the lift slightly to score pin
  lifts(20);
  lifts(MOTOR_BRAKE_HOLD);
  chassis.pid_drive_set(15_in, 60, true); //drive to goal for preload
  chassis.pid_wait();
  chassis.pid_drive_set(15_in, 10, true); //drive into goal to keep aligner straight
  pros::delay(10); //wait
  lift_auto(0); //lower the lift
  pros::delay(100); //wait
  lifts(MOTOR_BRAKE_HOLD);
  claw.set(true); //score preload by unclamping claw
//SCORE PIN/CUP
  chassis.pid_drive_set(-11_in, 60, true); //drive backwards to turn
  chassis.pid_wait();
  chassis.pid_turn_set(325, 60); //turn 
  chassis.pid_wait();
  chassis.pid_drive_set(20_in, 60, true); //drive further from goal
  chassis.pid_wait();
  //(Original code w/o dist sensor)
  // chassis.pid_turn_set(338, 60); //turn towards pin/cup
  // chassis.pid_wait();
  lift_auto(0); //bring the lift down to pick up pin/cup
  chassis.pid_turn_set(359, 23); //Turn to get ready to turn again towards pin/cup
  //DISTANCE SENSOR FOR TURNING TOWARDS PIN #1
  while (true) { 
    if (Dist.get() >= 200 && Dist.get() <= 400 && Dist.get() != 0) {
      break; 
    }
    pros::delay(20); 
  } 
  pros::delay(130); 
  //Reset heading for zero
  double current_heading = chassis.odom_theta_get();  // get the current heading
  chassis.pid_turn_set(current_heading, 0);           //Turn the current heading into 0
  pros::delay(200); //wait
  //DISTANCE SENSOR FOR DRIVING INTO PIN #1
  chassis.pid_drive_set(15_in, 30, true); //drive slowly towards pin/cup
  while (Dist.get() > 80 && Dist.get() != PROS_ERR) { //while loop got distance sensor
  pros::delay(20); // Check the sensor 50 times per second
  }
  claw.set(false); //clamp the pin/cup
  chassis.pid_drive_set(0_in, 0, true); //stop the chassis
  chassis.pid_wait_quick_chain();
  pros::delay(100); //wait
  lift_auto(100); //lift the cup/pin above the ground
  lifts(15); //force the lift up
  chassis.pid_turn_set(115, 60); //turn towards goalto score 
  chassis.pid_wait(); 
  chassis.pid_drive_set(11_in, 60, true); //drive to goal to score pin/cup
  chassis.pid_wait_until(1_in); //wait until you move an inch
  lift_auto(480); //lift the lift up to score pin
  lifts(15); //force the lift up
  chassis.pid_wait_quick_chain();
  pros::delay(10); //wait
  lift_auto(50); //bring the lift down to score pin/cup 1
  claw.set(true); //let go of the pin/cup
  chassis.pid_drive_set(-13_in, 60, true); //drive out of goal
  chassis.pid_wait();
  lift_auto(0); //bring the lift down to score pin/cup 2
  lifts(-5); //force the lift down
// SCORE PIN/CUP #2
  chassis.pid_turn_set(80, 60); //turn towards pin/cup 
  chassis.pid_wait();
  chassis.pid_drive_set(13_in, 60, true); //drive towards pin/cup
  chassis.pid_wait();
  //DISTANCE SENSOR FOR TURNING TOWARDS PIN #2
  chassis.pid_turn_set(359, 23); //turn the robot
  while (true) { //while loop for dist sensor
    if (Dist.get() >= 80 && Dist.get() <= 400 && Dist.get() != 0) {
      break; // Exit the loop immediately
    }
    pros::delay(20); 
  } 
  pros::delay(50); //wait
  //Reset heading for 0
  double currentheading = chassis.odom_theta_get();  
  chassis.pid_turn_set(currentheading, 110);           
  chassis.pid_wait();
  //DISTANCE SENSOR FOR DRIVING TOWARDS PIN #2
  chassis.pid_drive_set(15_in, 30); //drive slowly towards pin/cup
  while (Dist.get() > 100 && Dist.get() != PROS_ERR) { //dist sensor for driving 
  pros::delay(20); 
  }
  claw.set(false); //clamp the pin/cups
  chassis.pid_drive_set(0_in, 0, true); //stop the chassis
  chassis.pid_wait_quick_chain();
  pros::delay(50); //wait
  lift_auto(100); //lift the pin/cup above the ground
  lifts(15); //force the lift up
  chassis.pid_turn_set(210, 60); //turn towards goalto score 
  chassis.pid_wait();
  chassis.pid_drive_set(11_in, 60); //Drive into goal
  chassis.pid_wait_until(1_in); //wait until it drove one inch
  lift_auto(620); //lift the lift up to score pin
  lifts(15); //force the lift up
  lift_auto(200); //bring the lift down to score
  claw.set(true); //release the pin/cup
  lift_auto(0); //bring the lift all the way down


  //OLD CODE
  //Flip the toggle
  // claw.set(true); 
  // pros::delay(500); //wait half a sec
  // toggle1.move(127); //move the toggle motors at full speed
  // toggle2.move(127);
  // pros::delay(500); //wait half a sec
  // //Move towards goal
  // chassis.pid_drive_set(-13_in, 60, true); //drive forwards
  // chassis.pid_wait();
  // toggle1.move(0); //stop the toggle motors form spinning
  // toggle2.move(0);
  // chassis.pid_turn_set(90_deg, 60); //turn
  // chassis.pid_wait();
  // pros::delay(500); //wait half a sec
  // lift_auto(200); //lift the lift slightly to score pin
  // pros::delay(500); //wait half a sec
  // chassis.pid_drive_set(-18_in, 60, true); //drive forwards
  // chassis.pid_wait();
  // pros::delay(500); //wait half a sec
  // //Score preload
  // lift_auto(30); //lower the lift
  // lifts(-10);
  // pros::delay(500); //wait half a sec
  // claw.set(false); //score preload but unclamping claw
  // pros::delay(500); //wait half a sec
  // //Get pin/cup
  // chassis.pid_drive_set(15_in, 60); //drive forwards
  // chassis.pid_wait();
  //  chassis.pid_turn_set(200_deg, 60); //turn
  //   chassis.pid_wait();
  //  chassis.pid_drive_set(-17_in, 60); //drive forwards

  // chassis.pid_wait();
  //  pros::delay(500); //wait half a sec
  //   chassis.pid_turn_set(270_deg, 60); //turn
  //    chassis.pid_wait();
  //      chassis.pid_drive_set(-20_in, 60); //drive forwards
  //      chassis.pid_wait();
  // chassis.pid_turn_set(0_deg, 80); //turn
  // chassis.pid_wait();
  // pros::delay(500); //wait half a sec
  // lift_auto(0); //bring the lift down to score
  // lifts(-20);
  // chassis.pid_wait();
  // pros::delay(700); //wait half a sec
  // chassis.pid_drive_set(-14_in, 70, true); //drive toward pin/cup
  // chassis.pid_wait();
  // pros::delay(500); //wait half a sec
  // claw.set(true); //grab pin/cup
  // pros::delay(700); //wait 700 milliseconds
  // lift_auto(200); //lift the pin/cup slightly off ground
  // chassis.pid_wait();
  // pros::delay(500); //wait half a sec
  // //Score pin/cup
  // chassis.pid_drive_set(14_in, 70, true); //drive back
  // chassis.pid_wait();
  // pros::delay(500);//wait half a sec
  // chassis.pid_turn_set(90_deg, 70); //turn towards goal
  // chassis.pid_wait();
  //  pros::delay(500); //wait half a sec
  // lift_auto(500); //lift the lift slightly to score pin
  // pros::delay(500); //wait half a sec
  // chassis.pid_drive_set(-18_in, 70, true); //drive forwards
  // chassis.pid_wait();
  // pros::delay(500); //wait half a sec
  // //Score preload
  // lift_auto(300); //lower the lift
  // pros::delay(500); //wait half a sec
  // claw.set(false); //score preload but unclamping claw
  // pros::delay(500); //wait half a sec
}

void Skilssauton(){
  //FLIP TOGGLE
  claw.set(false); 
  toggle1.move_relative(-2000, 200);  //move 2000, velocity 2000
  toggle2.move_relative(-2000, 200); 
  pros::delay(450); //wait for toggles to finish turning
//SCORE PRELOAD
  lift_auto(400); //lift the lift slightly to score pin
  lifts(20);
  lifts(MOTOR_BRAKE_HOLD);
  chassis.pid_drive_set(15_in, 60, true); //drive to goal for preload
  chassis.pid_wait();
  chassis.pid_drive_set(15_in, 10, true); //drive into goal to keep aligner straight
  pros::delay(10); //wait
  lift_auto(0); //lower the lift
  pros::delay(100); //wait
  lifts(MOTOR_BRAKE_HOLD);
  claw.set(true); //score preload by unclamping claw
//SCORE PIN/CUP
  chassis.pid_drive_set(-11_in, 60, true); //drive backwards to turn
  chassis.pid_wait();
  chassis.pid_turn_set(325, 60); //turn 
  chassis.pid_wait();
  chassis.pid_drive_set(20_in, 60, true); //drive further from goal
  chassis.pid_wait();
  //(Original code w/o dist sensor)
  // chassis.pid_turn_set(338, 60); //turn towards pin/cup
  // chassis.pid_wait();
  lift_auto(0); //bring the lift down to pick up pin/cup
  chassis.pid_turn_set(359, 23); //Turn to get ready to turn again towards pin/cup
  //DISTANCE SENSOR FOR TURNING TOWARDS PIN #1
  while (true) { 
    if (Dist.get() >= 200 && Dist.get() <= 400 && Dist.get() != 0) {
      break; 
    }
    pros::delay(20); 
  } 
  pros::delay(130); 
  //Reset heading for zero
  double current_heading = chassis.odom_theta_get();  // get the current heading
  chassis.pid_turn_set(current_heading, 0);           //Turn the current heading into 0
  pros::delay(200); //wait
  //DISTANCE SENSOR FOR DRIVING INTO PIN #1
  chassis.pid_drive_set(15_in, 30, true); //drive slowly towards pin/cup
  while (Dist.get() > 80 && Dist.get() != PROS_ERR) { //while loop got distance sensor
  pros::delay(20); // Check the sensor 50 times per second
  }
  claw.set(false); //clamp the pin/cup
  chassis.pid_drive_set(0_in, 0, true); //stop the chassis
  chassis.pid_wait_quick_chain();
  pros::delay(100); //wait
  lift_auto(100); //lift the cup/pin above the ground
  lifts(15); //force the lift up
  chassis.pid_turn_set(115, 60); //turn towards goalto score 
  chassis.pid_wait(); 
  chassis.pid_drive_set(11_in, 60, true); //drive to goal to score pin/cup
  chassis.pid_wait_until(1_in); //wait until you move an inch
  lift_auto(480); //lift the lift up to score pin
  lifts(15); //force the lift up
  chassis.pid_wait_quick_chain();
  pros::delay(10); //wait
  lift_auto(50); //bring the lift down to score pin/cup 1
  claw.set(true); //let go of the pin/cup
  chassis.pid_drive_set(-13_in, 60, true); //drive out of goal
  chassis.pid_wait();
  lift_auto(0); //bring the lift down to score pin/cup 2
  lifts(-5); //force the lift down
// SCORE PIN/CUP #2
  chassis.pid_turn_set(80, 60); //turn towards pin/cup 
  chassis.pid_wait();
  chassis.pid_drive_set(13_in, 60, true); //drive towards pin/cup
  chassis.pid_wait();
  //DISTANCE SENSOR FOR TURNING TOWARDS PIN #2
  chassis.pid_turn_set(359, 23); //turn the robot
  while (true) { //while loop for dist sensor
    if (Dist.get() >= 80 && Dist.get() <= 400 && Dist.get() != 0) {
      break; // Exit the loop immediately
    }
    pros::delay(20); 
  } 
  pros::delay(50); //wait
  //Reset heading for 0
  double currentheading = chassis.odom_theta_get();  
  chassis.pid_turn_set(currentheading, 110);           
  chassis.pid_wait();
  //DISTANCE SENSOR FOR DRIVING TOWARDS PIN #2
  chassis.pid_drive_set(15_in, 30); //drive slowly towards pin/cup
  while (Dist.get() > 100 && Dist.get() != PROS_ERR) { //dist sensor for driving 
  pros::delay(20); 
  }
  claw.set(false); //clamp the pin/cups
  chassis.pid_drive_set(0_in, 0, true); //stop the chassis
  chassis.pid_wait_quick_chain();
  pros::delay(50); //wait
  lift_auto(100); //lift the pin/cup above the ground
  lifts(15); //force the lift up
  chassis.pid_turn_set(210, 60); //turn towards goalto score 
  chassis.pid_wait();
  chassis.pid_drive_set(11_in, 60); //Drive into goal
  chassis.pid_wait_until(1_in); //wait until it drove one inch
  lift_auto(620); //lift the lift up to score pin
  lifts(15); //force the lift up
  lift_auto(200); //bring the lift down to score
  claw.set(true); //release the pin/cup
  lift_auto(0); //bring the lift all the way down
  //ALIGNING TO MATCHLOADERS
  //PIN/CUP 4
  chassis.pid_drive_set(-11_in, 60); //Drive back
  chassis.pid_wait();
  chassis.pid_turn_set(40, 60); //turn towards wall to reset
  chassis.pid_wait();
  chassis.pid_drive_set(15_in, 30); 
  while (Dist.get() > 170 && Dist.get() != PROS_ERR) {  //dist sensor for wall position
  pros::delay(20); // Check the sensor 50 times per second
  }
  claw.set(true); //keep claw open
  chassis.pid_drive_set(0_in, 0, true); //stop the chassis
  chassis.pid_wait_quick_chain();
  pros::delay(50); //wait
  chassis.pid_turn_set(130, 60); //turn towards matchloader
  chassis.pid_wait();
  lift_auto(-100); //bring the lift down
  chassis.pid_drive_set(16.5_in, 40, true); //go into matchloader
  chassis.pid_wait(); 
  pros::delay(100); //wait
  claw.set(false); //clamp the pin/cup
  lift_auto(25); //lift the lift above the ground
  lifts(15); //force the lift up
  chassis.pid_drive_set(-17.5_in, 40, true); //back out of matchloader
  chassis.pid_wait(); //wait 
  lift_auto(100); //lift the lift up more above the ground
  lifts(15); //force the lift up
  chassis.pid_turn_set(210, 60); //turn towards goal
  chassis.pid_wait();
  lift_auto(700); //lift the lift up to score pin
  lifts(15); //force the lift up
  chassis.pid_drive_set(40_in, 40, true); //go into goal
  chassis.pid_wait(); //wait 
  lift_auto(400); //bring the lift down to score
  claw.set(true); //release the pin/cup
  //PIN/CUP 5:
  chassis.pid_drive_set(-11_in, 60); //Drive back
  chassis.pid_wait();
   lift_auto(0); //bring the lift down to score
  chassis.pid_turn_set(40, 60); //turn towards wall to reset
  chassis.pid_wait();
  chassis.pid_drive_set(17_in, 30); 
  while (Dist.get() > 170 && Dist.get() != PROS_ERR) {  //dist sensor for wall position
  pros::delay(20);
  }
  claw.set(true); //keep claw open
  chassis.pid_drive_set(0_in, 0, true); //stop the chassis
  chassis.pid_wait_quick_chain();
  pros::delay(50); //wait
  chassis.pid_turn_set(123, 60); //turn towards matchloader
  chassis.pid_wait();
  lift_auto(-100); //bring thel lift down to score
  lifts(-15); //force the lift down
  pros::delay(50); //wait
  chassis.pid_drive_set(16.5_in, 40, true); //go into matchloader
  chassis.pid_wait();  
  pros::delay(100); //wait
  claw.set(false); //clamp the pin/cup
  lift_auto(25); //lift the lift up above the ground
  lifts(15); //force the lift up
  chassis.pid_drive_set(-17.5_in, 40, true); //go out of the matchloader
  chassis.pid_wait(); 
  lift_auto(100); //lift the lift above the ground more
  lifts(15); //force the lift up
  chassis.pid_turn_set(210, 60); //turn towards goal
  chassis.pid_wait();
  lift_auto(900); //lift the lift up to score pin
  lifts(15); //force the lift up
  chassis.pid_drive_set(40_in, 40, true); //go into goal
  chassis.pid_wait(); //wait 
  lift_auto(500); //bring the lift down to score
  claw.set(true); //release the pin/cup
  //CUP/PIN 6
  chassis.pid_drive_set(-11_in, 60); //Drive back
  chassis.pid_wait();
   lift_auto(0); //bring the lift down to score
  chassis.pid_turn_set(40, 60); //turn towards wall to reset
  chassis.pid_wait();
  chassis.pid_drive_set(17_in, 30); 
  while (Dist.get() > 170 && Dist.get() != PROS_ERR) {  //dist sensor for wall position
  pros::delay(20); 
  }
  claw.set(true); //keep claw open
  chassis.pid_drive_set(0_in, 0, true); //stop the chassis
  chassis.pid_wait_quick_chain();
  pros::delay(50); //wait
  chassis.pid_turn_set(123, 60); //turn towards matchloader
  chassis.pid_wait();
  lift_auto(-100); //lift the lift down
  lifts(-15); //force the lift down
  pros::delay(50); 
  chassis.pid_drive_set(16.5_in, 40, true); //go into matchloader
  chassis.pid_wait(); 
  pros::delay(100); //wait
  claw.set(false); //clamp the pin/cup
  lift_auto(25); //lift the lift up above the ground
  lifts(15); //force the lift up
  chassis.pid_drive_set(-17.5_in, 40, true); //go out of matchloader
  chassis.pid_wait(); //wait 
  lift_auto(100); //lift the lift above the ground
  lifts(15); //force the lift up
  chassis.pid_turn_set(210, 60); //turn towards goal
  chassis.pid_wait();
  lift_auto(1100); //lift the lift up to score pin
  lifts(15); //force the lift up
  chassis.pid_drive_set(40_in, 40, true); //go into goal
  chassis.pid_wait(); //wait 
  chassis.pid_drive_set(-3, 40, true); //back up a bit 
  chassis.pid_wait(); //wait 
  lift_auto(500); //bring the lift down to score
  claw.set(true); //release the pin/cup
  //PIN 7 AND PARK
  chassis.pid_drive_set(-11_in, 60); //Drive back
  chassis.pid_wait();
   lift_auto(0); //bring the lift down to score
  chassis.pid_turn_set(40, 60); //turn towards wall to reset
  chassis.pid_wait();
  chassis.pid_drive_set(17_in, 30);
  while (Dist.get() > 170 && Dist.get() != PROS_ERR) {  //dist sensor for wall position
  pros::delay(20); 
  }
  claw.set(true); //keep claw open
  chassis.pid_drive_set(0_in, 0, true); //stop the chassis
  chassis.pid_wait_quick_chain();
  pros::delay(50); //wait
  chassis.pid_turn_set(123, 60); //turn towards matchloader
  chassis.pid_wait();
  lift_auto(-500); //force the lift down to the bottom
  lifts(-50); //force the lift down
  pros::delay(50); //wait
  chassis.pid_drive_set(16.5_in, 40, true); //go into matchloader
  chassis.pid_wait(); //wait 
  pros::delay(100); //wait
  claw.set(false); //clamp the pin/cup
  lift_auto(25); //lift the lift up above ground
  lifts(15); //force the lift up
  chassis.pid_drive_set(-16.5_in, 40, true); //go out of matchloader
  chassis.pid_wait(); //wait 
  lift_auto(100); //lift the lift up above ground
  lifts(15); //force the lift up
  chassis.pid_turn_set(255, 60); //turn towards middle goal
  chassis.pid_wait();
  lift_auto(300); //lift the lift up to score pin
  lifts(15); //force the lift up
  chassis.pid_drive_set(100_in, 60, true); //go into middle goal
  chassis.pid_wait(); //wait 
  lift_auto(0); //bring the lift down to score
  claw.set(true); //score the pin/cup
}



//right side auton
void rightsideauton() {
 
  //FLIP TOGGLE
  claw.set(false); 
  toggle1.move_relative(-2000, 200);  //move 2000, velocity 2000
  toggle2.move_relative(-2000, 200); 
  pros::delay(450); //wait for toggles to finish turning
//SCORE PRELOAD
  lift_auto(400); //lift the lift slightly to score pin
  lifts(20);
  lifts(MOTOR_BRAKE_HOLD);
  chassis.pid_drive_set(15_in, 60, true); //drive to goal for preload
  chassis.pid_wait();
  chassis.pid_drive_set(15_in, 10, true); //drive into goal to keep aligner straight
  pros::delay(10); //wait
  lift_auto(0); //lower the lift
  pros::delay(100); //wait
  lifts(MOTOR_BRAKE_HOLD);
  claw.set(true); //score preload by unclamping claw
//SCORE PIN/CUP
  chassis.pid_drive_set(-11_in, 60, true); //drive backwards to turn
  chassis.pid_wait();
  chassis.pid_turn_set(35, 60); //turn 
  chassis.pid_wait();
  chassis.pid_drive_set(20_in, 60, true); //drive further from goal
  chassis.pid_wait();
  lift_auto(0); //bring the lift down to pick up pin/cup
  chassis.pid_turn_set(0, 23); //Turn to get ready to turn again towards pin/cup
  //DISTANCE SENSOR FOR TURNING TOWARDS PIN #1
  while (true) { 
    if (Dist.get() >= 200 && Dist.get() <= 400 && Dist.get() != 0) {
      break; 
    }
    pros::delay(15); 
  } 
  pros::delay(130); 
  //Reset heading for zero
  double current_heading = chassis.odom_theta_get();  // get the current heading
  chassis.pid_turn_set(current_heading, 0);           //Turn the current heading into 0
  pros::delay(200); //wait
  //DISTANCE SENSOR FOR DRIVING INTO PIN #1
  chassis.pid_drive_set(15_in, 30, true); //drive slowly towards pin/cup
  while (Dist.get() > 80 && Dist.get() != PROS_ERR) { //while loop for distance
  pros::delay(50); 
  }
  claw.set(false); //clamp the pin/cup
  chassis.pid_drive_set(0_in, 0, true); //stop the chassis
  chassis.pid_wait_quick_chain();
  pros::delay(100); //wait
  lift_auto(100); //lift the cup/pin above the ground
  lifts(15); //force the lift up
  chassis.pid_turn_set(240, 60); //turn towards goalto score 
  chassis.pid_wait(); 
  chassis.pid_drive_set(8_in, 60, true); //drive to goal to score pin/cup
  chassis.pid_wait_until(1_in); //wait until you move an inch
  lift_auto(480); //lift the lift up to score pin
  lifts(15); //force the lift up
  chassis.pid_wait_quick_chain();
  pros::delay(10); //wait
  lift_auto(50); //bring the lift down to score pin/cup 1
  claw.set(true); //let go of the pin/cup
  chassis.pid_drive_set(-13_in, 60, true); //drive out of goal
  chassis.pid_wait();
  lift_auto(0); //bring the lift down to score pin/cup 2
  lifts(-5); //force the lift down
// SCORE PIN/CUP #2
  chassis.pid_turn_set(273, 60); //turn towards pin/cup 
  chassis.pid_wait();
  chassis.pid_drive_set(16_in, 60, true); //drive towards pin/cup
  chassis.pid_wait();
  //DISTANCE SENSOR FOR TURNING TOWARDS PIN #2
  chassis.pid_turn_set(320, 23); //turn the robot
  while (true) { //while loop for dist sensor
    if (Dist.get() >= 80 && Dist.get() <= 400 && Dist.get() != 0) {
      break; 
    }
    pros::delay(25); 
  } 
  pros::delay(75); //wait
  //Reset heading to 0
  double currentheading = chassis.odom_theta_get();  
  chassis.pid_turn_set(currentheading, 110);           
  chassis.pid_wait();
  //DISTANCE SENSOR FOR DRIVING TOWARDS PIN #2
  chassis.pid_drive_set(15_in, 30); //drive slowly towards pin/cup
  while (Dist.get() > 40 && Dist.get() != PROS_ERR) { //dist sensor for driving 
  pros::delay(100); 
  }
  claw.set(false); //clamp the pin/cup
  chassis.pid_drive_set(0_in, 0, true); //stop the chassis
  chassis.pid_wait_quick_chain();
  pros::delay(50); //wait
  lift_auto(100); //lift the pin/cup above the ground
  lifts(15); //force the lift up
  chassis.pid_turn_set(150, 60); //turn towards goalto score 
  chassis.pid_wait();
  chassis.pid_drive_set(11_in, 60); //Drive into goal
  chassis.pid_wait_until(1_in); //wait until it drove one inch
  lift_auto(620); //lift the lift up to score pin
  lifts(15); //force the lift up  
  lift_auto(200); //bring the lift down to score
  claw.set(true); //release the pin/cup
  lift_auto(0); //bring the lift all the way down

  //OLD CODE
  //Move towards goal
  // chassis.pid_drive_set(-14_in, 60, true); //drive forwards
  // chassis.pid_wait();
  // toggle1.move(0); //stop the toggle motors form spinning
  // toggle2.move(0);
  // chassis.pid_turn_set(270_deg, 60); //turn
  // chassis.pid_wait();
  // pros::delay(500); //wait half a sec
  // lift_auto(200); //lift the lift slightly to score pin
  // pros::delay(500); //wait half a sec
  // chassis.pid_drive_set(-18_in, 60, true); //drive forwards
  // chassis.pid_wait();
  // pros::delay(500); //wait half a sec
  // //Score preload
  // lift_auto(30); //lower the lift
  // pros::delay(500); //wait half a sec
  // claw.set(false); //score preload but unclamping claw
  // pros::delay(500); //wait half a sec
  // //Get pin/cup
  // chassis.pid_drive_set(17_in, 60); //drive forwards
  // chassis.pid_wait();
  // chassis.pid_turn_set(0_deg, 80); //turn
  // chassis.pid_wait();
  // pros::delay(500); //wait half a sec
  // lift_auto(0); //bring the lift down to score
  //   lifts(-20);
  // chassis.pid_wait();
  // pros::delay(700); //wait half a sec
  // chassis.pid_drive_set(-15_in, 70, true); //drive toward pin/cup
  // chassis.pid_wait();
  // pros::delay(500); //wait half a sec
  // claw.set(true); //grab pin/cup
  // pros::delay(700); //wait 700 milliseconds
  // lift_auto(200); //lift the pin/cup slightly off ground
  // chassis.pid_wait();
  // pros::delay(500); //wait half a sec
  // //Score pin/cup
  // chassis.pid_drive_set(14_in, 70, true); //drive back
  // chassis.pid_wait();
  // pros::delay(500);//wait half a sec
  // chassis.pid_turn_set(270_deg, 70); //turn towards goal
  // chassis.pid_wait();
  //  pros::delay(500); //wait half a sec
  // lift_auto(500); //lift the lift slightly to score pin
  // pros::delay(500); //wait half a sec
  // chassis.pid_drive_set(-18_in, 70, true); //drive forwards
  // chassis.pid_wait();
  // pros::delay(500); //wait half a sec
  // //Score preload
  // lift_auto(300); //lower the lift
  // pros::delay(500); //wait half a sec
  // claw.set(false); //score preload but unclamping claw
  // pros::delay(500); //wait half a sec
}
///
// Drive Example
///
void drive_example() {
    // The first parameter is target inches
    // The second parameter is max speed the robot will drive at
    // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
    // for slew, only enable it when the drive distance is greater than the slew distance + a few inches
// move forward 24 inches
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true); 
  chassis.pid_wait();
//move back 12 inches
  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
//move back another 12 inches
  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

//Lift PID
void lift() {
   lift_auto(400); //lift the lift up to score pin
}



///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .