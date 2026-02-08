#include "main.h"
#include "EZ-Template/intake.hpp"
#include "EZ-Template/outtake.hpp"
#include "EZ-Template/pneumatics.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// //Mahyar Variables (Mainly for PID as of rn)
// double c_mass = 7.39;
// //Drive Constants
// double kp_drive = 20.0; double kd_drive = 2 * sqrt(c_mass * kp_drive);
// //Heading Constants
// double kp_heading = 11.0; double kd_heading = 2 * sqrt(c_mass * kp_heading);
// //Turn Constants
// double kp_turn = 3.0; double kd_turn = 2 * sqrt(c_mass * kp_turn);
// //Swing Constants
// double kp_swing = 6.5; double kd_swing = 2 * sqrt(c_mass * kp_swing);
// //Odom Constants
// double kp_odom = 6.5; double kd_odom = 2 * sqrt(c_mass * kp_odom);
// //Boomerang Constants
// double kp_boom = 5.8; double kd_boom = 2 * sqrt(c_mass * kp_boom);


// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions
  // chassis.pid_drive_constants_set(kp_drive, 0.0, kd_drive);         // Fwd/rev constants, used for odom and non odom motions
  // chassis.pid_heading_constants_set(kp_heading, 0.0, kd_heading);        // Holds the robot straight while going forward without odom
  // chassis.pid_turn_constants_set(kp_turn, 0.05, kd_turn, 15.0);     // Turn in place constants
  // chassis.pid_swing_constants_set(kp_swing, 0.0, kd_swing);           // Swing constants
  // chassis.pid_odom_angular_constants_set(kp_odom, 0.0, kd_odom);    // Angular control for odom motions
  // chassis.pid_odom_boomerang_constants_set(kp_boom, 0.0, kd_boom);  // Angular control for boomerang motions

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

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

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

void basic_run(){

  chassis.imu.set_heading(180);

  pros::delay(5000);

  rake_open();

  pros::delay(500);

  intake_speed(127);
  outtake_speed(127);

  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();

  rake_close();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(15_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // chassis.pid_drive_set(22.5_in, DRIVE_SPEED);
  // chassis.pid_wait();

  chassis.pid_drive_set(3_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -90_deg, SWING_SPEED, 57);
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(30_in, DRIVE_SPEED);
  chassis.pid_wait();

  pros::delay(500);

  angle_open();

  intake_speed(-127);
  outtake_speed(-127);

  pros::delay(3000);

  intake_speed(0);
  outtake_speed(0);

  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);
}

void charity_new_run(){

  pros::delay(5000);

  rake_open();

  pros::delay(500);

  intake_speed(127);
  outtake_speed(127);

  chassis.pid_drive_set(-7_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(4_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-5_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  pros::delay(1000);

  rake_close();

  chassis.pid_drive_set(-2_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // chassis.pid_drive_set(22.5_in, DRIVE_SPEED);
  // chassis.pid_wait();

  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(30_in, DRIVE_SPEED);
  chassis.pid_wait();

  intake_speed(0);
  outtake_speed(0);

  // chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // chassis.pid_drive_set(35_in, DRIVE_SPEED);
  // chassis.pid_wait();
}

void testing_park_and_clear(){

  chassis.drive_imu_reset();
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.odom_reset();

  //Waits 5 seconds to allow for Justin's robot to gain enough distance
  //before starting its path to allow for no collision
  pros::delay(5000);

  //Drops down rake mech, effectively grabbing all blocks
  rake_open();

  //Closes the block cover to keep all blocks inside robot
  ball_close();

  //Waits 500 msec to allow for proper encloser of blocks
  pros::delay(500);

  //Starts intaking blocks
  intake_speed(127);
  outtake_speed(127);

  //Drives backwards 5 inches
  chassis.pid_drive_set(-4_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Turns to face right side of field
  //if on Red Alliance
  chassis.pid_turn_set(-90_deg, 50);
  chassis.pid_wait();

  chassis.pid_drive_set(20_in, 60);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(5_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg, DRIVE_SPEED);
  chassis.pid_wait();

  //Bring rake up to allow for better manueverability
  //Open block cover to allow blocks to funnel out
  rake_close();
  // ball_open();

  //Drives forward 60 inches
  //Allows robot to line up aainst the wall
  chassis.pid_drive_set(50_in, 65);
  chassis.pid_wait();

  //Drives backwards 16.25 inches
  //Allows robot to line up with matchloader
  //in the Y-axis and to pick up blocks along wall
  chassis.pid_drive_set(-14.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Turn to face long goal
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  //Open the block cover to score in long goal
  ball_open();
  angle_open();

  //Drive Forward 12.5 inches to long goal 
  chassis.pid_drive_set(14_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Wait 5 sec to score into long goal
  pros::delay(5000);

  //Drive back 5 inches
  chassis.pid_drive_set(-5_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Turn to face matchloader
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  //Shoot down rake mech
  rake_open();
  ball_close();

  pros::delay(1000);

  //Keep intaking
  intake_speed(127);
  outtake_speed(127);

  //Drive into the match loader
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-2_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-2_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-2_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-2_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Wait 4 sec to intake all match load balls
  pros::delay(1000);

  //Drive backwards 15 inches
  chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Turn to face long goal again
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  //Close the rake mech
  rake_close();

  //Drive 10 inches forward to line up with long goal
  chassis.pid_drive_set(9_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Open block cover to score
  ball_open();

  //Wait 5 seconds to score into long goal
  pros::delay(5000);

  //Drive 10 inches back to line up with prev spot
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Turn towards the left side of the field
  //From red alliance position
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  //Drive forward 48 inches to line up with the parking zone
  chassis.pid_drive_set(43_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Close rake mech to allow for better turning
  rake_close();

  //Turn to face parking zone
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  //Drive back for windup
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Drive into parking zone
  chassis.pid_drive_set(30_in, DRIVE_SPEED);
  chassis.pid_wait();

  //Stop intake
  intake_speed(0);
  outtake_speed(0);
}

void odom_testing(){
  chassis.drive_imu_reset();
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.odom_reset();
  chassis.odom_xyt_set(0_in, -48_in, 180_deg);

  //Waits 5 seconds to allow for Justin's robot to gain enough distance
  //before starting its path to allow for no collision
  pros::delay(5000);

  //Drops down rake mech, effectively grabbing all blocks
  rake_open();

  //Closes the block cover to keep all blocks inside robot
  ball_close();

  //Waits 500 msec to allow for proper encloser of blocks
  pros::delay(500);

  //Starts intaking blocks
  intake_speed(127);
  outtake_speed(127);

  chassis.pid_odom_smooth_pp_set({{{0_in, -30_in, 180_deg}, rev, DRIVE_SPEED}}, true);
  chassis.pid_wait();

  //Was 65_in on X for second index
  chassis.pid_odom_smooth_pp_set({{{36_in, -50_in, 90_deg}, fwd, 80},
                                  {{60_in, -50_in, 90_deg}, fwd, 65}}, true);
  chassis.pid_wait_until_index_started(1);

  //Bring rake up to allow for better manueverability
  //Open block cover to allow blocks to funnel out
  rake_close();
  chassis.pid_wait();

  ////////////////////////////////////////////
  chassis.pid_drive_set(5_in, DRIVE_SPEED);
  chassis.pid_wait();
  ////////////////////////////////////////////

  chassis.odom_xyt_set(65_in, -50_in, 90_deg);

  pros::delay(500);

  chassis.pid_odom_smooth_pp_set({{{52.5_in, -48_in, 90_deg}, rev, DRIVE_SPEED},
                                  {{52.5_in, -35_in, 0_deg}, fwd, 70}}, true);
  chassis.pid_wait_until_index_started(1);
  angle_open();
  chassis.pid_wait();

  ball_open();

  pros::delay(3000);

  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();

  rake_open();
  ball_close();

  // chassis.pid_odom_ptp_set({{50_in, -59.5_in, 179_deg}, fwd, DRIVE_SPEED}, true);
  // chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(20_in, 75);
  chassis.pid_wait();

  pros::delay(500);

  chassis.pid_drive_set(-2_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-2_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-2_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-2_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();

  rake_close();

  intake_speed(0);
  outtake_speed(0);

  ///////////////////////////////////////////////////////////////////////////

  chassis.pid_turn_set(0_deg, 60);
  chassis.pid_wait();

  chassis.pid_drive_set(10_in, 70);
  chassis.pid_wait();

  //Was 1.75 inches prior to 2.5 inches
  chassis.pid_drive_set(2.75_in, 50, false);
  chassis.pid_wait();

  // //Point to go to long goal again
  // chassis.pid_odom_ptp_set({{50_in, -42_in, 0_deg}, fwd, 70}, true);
  // chassis.pid_wait();

  intake_speed(127);
  outtake_speed(127);
  ball_open();

  pros::delay(4000);

  chassis.pid_drive_set(1_in, 50, false);
  chassis.pid_wait();

  ///////////////////////////////////////////////////////////////////////////

  // //Point to go to Low Goal
  // chassis.pid_odom_ptp_set({{17_in, -17_in, -45_deg}, fwd, 70}, true);
  // chassis.pid_wait();

  // chassis.pid_drive_set(-5_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // intake_speed(-127);
  // outtake_speed(-127);
  // pros::delay(4000);

  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();

  angle_close();

  chassis.pid_odom_smooth_pp_set({{{48_in, -45_in, 180_deg}, fwd, DRIVE_SPEED},
                                  {{5_in, -40_in, 180_deg}, fwd, DRIVE_SPEED},
                                  {{5_in, -72_in, 180_deg}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait_until_index_started(2);
  ball_close();
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(20_in, DRIVE_SPEED);
  chassis.pid_wait();

  intake_speed(0);
  outtake_speed(0);
  
}

void match_auton(){
  // y = -50, x = 18, theta = 90
  chassis.drive_imu_reset();
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.odom_reset();
  chassis.odom_xyt_set(18_in, -50_in, 90_deg);

  // chassis.pid_drive_set(26_in, DRIVE_SPEED);
  // chassis.pid_wait_quick();

  // chassis.pid_turn_set(180_deg, TURN_SPEED);
  // chassis.pid_wait();

  // chassis.pid_odom_ptp_set({{47_in, -50_in, 90_deg}, fwd, 65}, true);
  // chassis.pid_wait();

  chassis.pid_odom_ptp_set({{45.5_in, -50_in, 90_deg}, fwd, 65}, true);
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, 60);
  chassis.pid_wait();

  chassis.pid_drive_set(-5_in, DRIVE_SPEED);
  chassis.pid_wait();

  rake_open();
  ball_close();
  intake_speed(127);
  outtake_speed(127);

  chassis.pid_drive_set(24_in, 60);
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();

  rake_close();
  angle_open();
  // intake_speed(0);
  // outtake_speed(0);

  chassis.pid_turn_set(1.75_deg, 70);
  chassis.pid_wait();

  chassis.pid_drive_set(14_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.75_in, 60);
  chassis.pid_wait();

  ball_open();
  // pros::delay(500);
  // intake_speed(127);
  // outtake_speed(127);
  pros::delay(1750);
  
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();

  intake_speed(0);
  outtake_speed(0);

  chassis.pid_turn_set(-90_deg, 80);
  chassis.pid_wait();

  intake_speed(-127);
  outtake_speed(-127);
  pros::delay(100);
  intake_speed(127);
  outtake_speed(127);
  pros::delay(1000);

  chassis.pid_drive_set(-2.5_in, 60, false);
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, 60);
  chassis.pid_wait();

  rake_open();
  ball_close();
  intake_speed(127);
  outtake_speed(127);

  chassis.pid_drive_set(-7.5_in, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(24_in, 55);
  chassis.pid_wait();

  chassis.pid_drive_set(2_in, 70);
  chassis.pid_wait();

  chassis.pid_drive_set(-2_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED, false);
  chassis.pid_wait();

  //was 3000
  pros::delay(1500);

  chassis.pid_drive_set(-10_in, 75);
  chassis.pid_wait();

  rake_close();
  angle_open();
  // intake_speed(0);
  // outtake_speed(0);

  // chassis.pid_turn_set(4.5_deg, 60);
  // chassis.pid_wait();
  chassis.pid_turn_set(3_deg, 60);
  chassis.pid_wait();

  intake_speed(-127);
  outtake_speed(-127);
  pros::delay(100);
  intake_speed(0);
  outtake_speed(0);

  chassis.pid_drive_set(13_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-1.5_in, 60);
  chassis.pid_wait();

  ball_open();
  intake_speed(127);
  outtake_speed(127);
  pros::delay(500);
  // intake_speed(127);
  // outtake_speed(127);
  pros::delay(2500);

  angle_close();

  // chassis.pid_drive_set(-10_in, 80);
  // chassis.pid_wait();

  // intake_speed(0);
  // outtake_speed(0);

  pros::delay(200);
  angle_open();
}

// . . .