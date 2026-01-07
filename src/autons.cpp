#include "main.h"
using namespace okapi;   
#include "subsystems.hpp"
/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int TURN_SPEED = 90;
const int TURN_SPEED_SLOW = 60;
const int SWING_SPEED = 110;
//Drive speeds
const int DRIVE_SPEED = 110;
const int DRIVE_SPEED_SLOW = 40;
const int DRIVE_SPEED_MEDIUM = 65;
 


///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.001, 90.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
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
    chassis.pid_drive_set(-24_in, 127);
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


// Custom Helper Functions
void drive(QLength distance, int speed = DRIVE_SPEED, bool slew = true) {
  chassis.pid_drive_set(distance, speed, slew);
  chassis.pid_wait();
}

void turn(okapi::QAngle angle, int speed = TURN_SPEED) {
  chassis.pid_turn_set(angle, speed);
  chassis.pid_wait();
}

void swingAbsLeft(double deg, int speed = 90) {
  chassis.pid_swing_set(ez::LEFT_SWING, deg * 1_deg, speed);
  chassis.pid_wait();
}

void swingAbsRight(double deg, int speed = 90) {
  chassis.pid_swing_set(ez::RIGHT_SWING, deg * 1_deg, speed);
  chassis.pid_wait();
}

void arcRightAbs(double deg, int turnSpeed = 90, int insideSpeed = 30) {
  chassis.pid_swing_set(ez::RIGHT_SWING, deg * 1_deg, turnSpeed, insideSpeed);
  chassis.pid_wait();
}

void arcLeftAbs(double deg, int turnSpeed = 90, int insideSpeed = 30) {
  chassis.pid_swing_set(ez::LEFT_SWING, deg * 1_deg, turnSpeed, insideSpeed);
  chassis.pid_wait();
}

void correct(double desired_dist, double precision) {
  // could loop this or just call it twice
  double dist = front_distance.get(); // 15.5
  double error = desired_dist - dist;  // 10-15.5
  if (std::abs(error) < precision) { // 5.5 < 0.1
    return;
  }
  QLength qDist = error * okapi::millimeter;

  drive(qDist);
}

// set this around 20-30 for gentle matchloading
void matchload(int power) {
  for (auto &m : chassis.left_motors) {
    m.move(power);
  }
  for (auto &m : chassis.right_motors) {
    m.move(power);
  }
}

  // bool collectorExtended = false;
// // Extend the block collector
  // collectorExtended = true;
  // block_collector.set_value(collectorExtended);
  // pros::delay(500);  // Wait 0.5 seconds for pneumatic action

// // Retract the block collector
  // collectorExtended = false;
  // block_collector.set_value(collectorExtended);
  // pros::delay(300);

  // . . .
// Make your own autonomous functions here!
// . . .
void MatchAutonAWP() {
  bool collectorExtended = false;
  chassis.imu.tare_rotation();
  chassis.drive_angle_set(90_deg);

  // ===== Initial Drive & Collector Deploy =====
  drive(31_in, 127);
  collectorExtended = true;
  block_collector.set_value(collectorExtended);
  // collect blocks
  combine.move(-127);
  intake.move(127);
  turn(180_deg, 127);
  drive(14_in, 60);
  // Matchload
  matchload(25);
  pros::delay(2000);
  matchload(0);
  combine.move(-35);
  intake.move(40);
  // ===== Score First Blocks=====
  drive(-12_in, 127);
  collectorExtended = false;
  block_collector.set_value(collectorExtended);
  turn(363_deg, 127);
  drive(12.25_in, 127);
  hood.move(-127);
  intake.move(127);
  combine.move(-127);
  pros::delay(1000);
  // ===== Back Up and Collect Blocks for Upper Middle =====
  drive(-22_in, 127);
  turn(-45_deg, 127);
  combine.move(-127);
  intake.move(127);
  drive(42_in, 127);
  combine.move(-30);
  turn(-90_deg,127);
  drive(44_in, 127);
  block_collector.set_value(true);
  block_collector.set_value(collectorExtended);
  drive(4_in, 70);
  turn(45_deg, 127);
  combine.move(-127);
  intake.move(127);
  hood.move(127);
  drive(15.5_in, 127);
  pros::delay(200);
  intake.move(0);
  combine.move(0);
// ===== Collect and Score Upper Left Blocks =====
  drive(-51_in, 127);
  turn(-180_deg, 127);
  // Collect Second Set Blocks
  intake.move(127);
  combine.move(-127);
  drive(15_in, DRIVE_SPEED_MEDIUM);
  // Matchload
  matchload(25);
  pros::delay(2000);
  matchload(0);
  combine.move(-35);
  intake.move(40);
  drive(-14_in, 127);
  collectorExtended = false;
  block_collector.set_value(collectorExtended);
  turn(360_deg, 127);
  // ===== Score Second Set Blocks =====
  drive(9.5_in);
  intake.move(127);
  combine.move(-127);
  hood.move(-127);
  pros::delay(3000);
}

void QualAutonR() {
  bool collectorExtended = false;
  chassis.imu.tare_rotation();
  chassis.drive_angle_set(90_deg);

  // Initial Drive & Collector Deploy 
  collectorExtended = true;
  block_collector.set_value(collectorExtended);
  pros::delay(1500);
  drive(30.5_in, 127);
  // collect blocks
  combine.move(-65);
  intake.move(127);
  turn(180_deg, 70);
  drive(14_in, 70);
  // Matchload
  matchload(25);
  pros::delay(200);
  matchload(0);
  combine.move(-60);
  intake.move(40);
// Backup to score first blocks
  drive(-15_in);
  turn(363_deg);
  collectorExtended = false;
  block_collector.set_value(collectorExtended);
  pros::delay(300);
  // ===== Score Second Set Blocks =====
  drive(11_in);
  intake.move(127);
  combine.move(-127);
  hood.move(-125);
  pros::delay(3000);
  drive(-8_in);
descore_mech.set_value(true);  // extend / go up
  pros::delay(300);
  turn(65_deg);
  drive(15_in);
  turn(360_deg);
  // Collect Second Set Blocks
  pros::delay(100);
  drive(8_in);
  descore_mech.set_value(false);  // extend / go up
  drive(15.5_in,127);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

}
 
void MatchAutonR() {
 bool collectorExtended = false;
  chassis.slew_swing_set(true);  // Enables global slew
  chassis.imu.tare_rotation();

 // ===== Path to First Blocks =====
  intake.move(100);
  combine.move(-40);
drive(10_in);              
turn(30_deg);
drive(17.5_in, DRIVE_SPEED_SLOW);
pros::delay(350);
  // ===== Score First Block =====
 turn(-45_deg);
 drive(14.75_in, DRIVE_SPEED);
 intake.move(-127);
combine.move(127);
 pros::delay(2000);
 combine.move(10);
 intake.move(20);
  // ===== Drive to Matchloader =====
  drive(-49.25_in);
  collectorExtended = true;
  block_collector.set_value(collectorExtended);
  pros::delay(800);
  turn(-180_deg);
  // ===== Collect Second Set Blocks =====
 intake.move(127);
 combine.move(-127);
  drive(20_in,70);
    // Wiggle
  chassis.pid_drive_set(-3, 127);    
  pros::delay(50);

  chassis.pid_drive_set(3.25, 127);
  pros::delay(150);
  combine.move(-25);
  intake.move(60);
  drive(-15_in);
  turn(360_deg);
  // ===== Retract Collector =====
  collectorExtended = false;
  block_collector.set_value(collectorExtended);
  pros::delay(300);
  // ===== Score Second Set Blocks =====
  drive(10.25_in);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

  intake.move(127);
  combine.move(-127);
  hood.move(-127);
  pros::delay(3500);
}

void QualAutonL() {
  bool collectorExtended = false;

  // ===== Collect First Set Blocks =====
  intake.move(80);
  combine.move(-80);
  drive(28_in, DRIVE_SPEED_SLOW);
  intake.move(40);
  combine.move(-40);
  pros::delay(250);

  // ===== Deploy Collector =====
  collectorExtended = true;
  block_collector.set_value(collectorExtended);
  pros::delay(800);

  // ===== Score First Block =====
  turn(65.5_deg);
  drive(14_in, DRIVE_SPEED_MEDIUM);
  hood.move(90);
  intake.move(127);
  combine.move(-127);
  pros::delay(3000);
  intake.move(0);
  combine.move(0);

  // ===== Drive to Matchloader =====
  drive(-48_in);
  turn(200_deg);

  // ===== Collect Second Set Blocks =====
  intake.move(127);
  combine.move(-127);
  drive(17_in, DRIVE_SPEED_MEDIUM);
  drive(-3_in);
  drive(5_in);
  drive(-3_in);
  combine.move(0);
  intake.move(20);
  hood.move(90);

  drive(-14_in);
  pros::delay(2500);
  // ===== Retract Collector =====
  collectorExtended = false;
  block_collector.set_value(collectorExtended);
  pros::delay(800);

  // ===== Score Second Set Blocks =====
  turn(360_deg);
  drive(8.67_in, DRIVE_SPEED);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  intake.move(127);
  combine.move(-100);
  hood.move(-127);
  pros::delay(3000);
}

void MatchAutonL() {
  bool collectorExtended = false;
  chassis.imu.tare_rotation();
  chassis.drive_angle_set(-90_deg);

  // Initial Drive & Collector Deploy 
  collectorExtended = true;
  block_collector.set_value(collectorExtended);
  pros::delay(200);
  drive(30_in, 127);
  // collect blocks
  combine.move(-65);
  intake.move(127);
  turn(180_deg, 127);
  drive(14_in, 70);
  // Matchload
  matchload(25);
  pros::delay(200);
  matchload(0);
  combine.move(-60);
  intake.move(40);
// Backup to score first blocks
  drive(-15_in);
  turn(360_deg);
  collectorExtended = false;
  block_collector.set_value(collectorExtended);
  pros::delay(300);
  // ===== Score Second Set Blocks =====
  drive(11_in);
  intake.move(127);
  combine.move(-127);
  hood.move(-127);
  pros::delay(2500);
  drive(-8_in);
descore_mech.set_value(true);  // extend / go up
pros::delay(300);
  pros::delay(300); // wait for airet_value(true);
  turn(65_deg);
  drive(12_in);
  turn(360_deg);
  // Collect Second Set Blocks
  pros::delay(100);
  drive(8_in);
  descore_mech.set_value(false);  // extend / go up
  drive(12_in,127);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
}

void SkillsAutonPark() {
chassis.pid_drive_set(-10_in, 120, true);
chassis.pid_wait();
chassis.pid_drive_set(60_in, 120, true);
chassis.pid_wait();
chassis.pid_drive_set(-10_in, 120, true);
chassis.pid_wait();
}

void SkillsAuton1() {
  chassis.drive_angle_set(90_deg);
  bool collectorExtended = false;

  // ===== Initial Drive & Collector Deploy =====
  drive(31_in, DRIVE_SPEED);
  collectorExtended = true;
  block_collector.set_value(collectorExtended);
  pros::delay(500);

  // ===== Collect Blocks =====
  combine.move(-50);
  intake.move(100);
  turn(180_deg);
  drive(14_in, 60);

  // Wiggle
chassis.pid_drive_set(-3, 127);    
  pros::delay(200);

  chassis.pid_drive_set(4.5, 127);
  pros::delay(200);

  chassis.pid_drive_set(-3, 127);
  pros::delay(200);

  drive(3_in, 70);

  chassis.pid_drive_set(4.5, 127);

  chassis.pid_drive_set(-3, 45);    
  pros::delay(200);

  chassis.pid_drive_set(4.5, 45);
  pros::delay(200);

  chassis.pid_drive_set(-3, 45);
  pros::delay(200);

  chassis.pid_drive_set(4.5, 45);
  pros::delay(800);
  pros::delay(1000);
  combine.move(-10);
  intake.move(127);

  // ===== Back Up To Sore First Blocks=====
  drive(-11.5_in);
  collectorExtended = false;
  block_collector.set_value(collectorExtended);
  pros::delay(800);
  turn(365_deg);
  drive(14_in);
  hood.move(-127);
  intake.move(127);
  combine.move(-127);
  pros::delay(4250);
  //=====Move to Second Goal=====
  intake.move(-127);
  combine.move(-127);

  drive(-11_in);
  turn(90_deg);
  collectorExtended = true;
  block_collector.set_value(collectorExtended);
  drive(-120_in, DRIVE_SPEED);
  chassis.drive_angle_set(90_deg);
  drive(17_in);
  turn(180_deg);

  //=====Collect Second Goal Blocks=====
  pros::delay(200);
  combine.move(-65);
  intake.move(100);
  turn(180_deg);
  drive(35_in, 60);
  chassis.pid_drive_set(-3, 127);    
  pros::delay(200);

  chassis.pid_drive_set(4.5, 127);
  pros::delay(200);

  chassis.pid_drive_set(-1, 127);
  pros::delay(200);

  drive(3_in, 70);

  chassis.pid_drive_set(4.5, 127);

  chassis.pid_drive_set(-1, 45);    
  pros::delay(200);

  chassis.pid_drive_set(4.5, 45);
  pros::delay(200);

  chassis.pid_drive_set(-1, 45);
  pros::delay(200);

  chassis.pid_drive_set(4.5, 45);
  pros::delay(800);
  combine.move(0);
  intake.move(60);
  hood.move(0);

  // ===== Back Up To Score Second Blocks=====
  drive(-12.25_in);
  collectorExtended = false;
  block_collector.set_value(collectorExtended);
  pros::delay(800);
  turn(360_deg);
  drive(13_in);
  hood.move(-127);
  intake.move(127);
  combine.move(-127);
  pros::delay(4250);

//=====Collect 4 Left Blocks=====
  intake.move(-127);
  drive(-12_in);
  turn(45_deg);
  intake.move(127);
  combine.move(-127);
  drive(23_in, DRIVE_SPEED);
  collectorExtended = true;
  block_collector.set_value(collectorExtended);
  drive(7_in, 30);
  pros::delay(500);
  intake.move(15);
  combine.move(-15);
  hood.move(0);
  drive(-30_in);
  collectorExtended = false;
  block_collector.set_value(collectorExtended);
  turn(360_deg);
  drive(13.5_in);
  hood.move(-127);
  intake.move(127);
  combine.move(-127);
  pros::delay(4000);

  //=====Collect 4 Right Blocks=====
  drive(-11_in);
  turn(-270_deg);
  collectorExtended = true;
  block_collector.set_value(collectorExtended);
  drive(68_in, DRIVE_SPEED);
  collectorExtended = false;
  block_collector.set_value(collectorExtended);
  turn(360_deg);
  drive(14_in);
  drive(7_in, 30);
  collectorExtended = true;
  block_collector.set_value(collectorExtended);
  intake.move(127);
  combine.move(-127);
  hood.move(0);
  pros::delay(400);
  intake.move(0);
  combine.move(0);
  drive(-21_in);
  collectorExtended = false;
  block_collector.set_value(collectorExtended);
  turn(-270_deg);
  drive(28_in);
  turn(360_deg);
  drive(10_in);
  hood.move(-127);
  intake.move(127);
  combine.move(-127);
  pros::delay(4000);
   // =====Park Robot=====
  drive(-10_in);
  turn(-90_deg);
  drive(47_in, 100);
  turn(180_deg);
  hood.move(127);
  intake.move(127);
  combine.move(-127);
  drive(90_in, 127);
  pros::delay(4000);
}

void SkillsAuton2() {
  chassis.drive_angle_set(90_deg);
  bool collectorExtended = false;

  // ===== Initial Drive & Collector Deploy =====
  drive(31.25_in, DRIVE_SPEED);
  collectorExtended = true;
  block_collector.set_value(collectorExtended);
  pros::delay(500);

  // Collect Blocks 
  combine.move(-60);
  intake.move(100);
  turn(180_deg);
  drive(13.5_in, 60);

  // correct(10, 1);

   // Matchload
chassis.pid_drive_set(-1, 127);    
  pros::delay(200);

  chassis.pid_drive_set(4.5, 127);
  pros::delay(200);
  matchload(25);
  pros::delay(3000);
  matchload(0);
  intake.move(40);
  combine.move(0);

  // Back Up
  drive(-20_in, 127);
  
  collectorExtended = false;
  block_collector.set_value(collectorExtended);
  pros::delay(800);
  hood.move(0);
  intake.move(127);
  combine.move(0);
  turn(55_deg);

  // =====Move to Next Goal=====
  drive(17_in,127);
  turn(365_deg);
  drive(68_in, 127);
  turn(-90_deg);
  drive(-4.5_in);
  chassis.drive_angle_set(-90_deg);
  drive(17_in);
  turn(180_deg);
  drive(8.75_in);

  hood.move(-127);
  intake.move(127);
  combine.move(-110);
  pros::delay(4250);

  hood.move(0);
  intake.move(0);
  combine.move(0);

  // Collect Other Blocks
  drive(-12_in);    
  collectorExtended = true;
  block_collector.set_value(collectorExtended);    
  pros::delay(800); 
  turn(360_deg);
  hood.move(-127);
  intake.move(127);
  combine.move(-65);         
  drive(15_in, 60);

   // Matchload
  matchload(25);
  pros::delay(3000);
  matchload(0);
  hood.move(0);
  intake.move(127);
  combine.move(0);
  // score
  drive(-12_in);
  turn(180_deg);
  collectorExtended = false;
  block_collector.set_value(collectorExtended);
  pros::delay(800);
  drive(13.5_in);
  hood.move(-127);
  intake.move(127);
  combine.move(-110);
  pros::delay(4250);

  // =====Move to Left Side Match Load=====
  drive(-12_in, 127);
  turn(-90_deg);
  combine.move(127);
  intake.move(-127);
  drive(94_in, 127);
  turn(360_deg);
 
  collectorExtended = true;
  block_collector.set_value(collectorExtended);
  pros::delay(1000);
  // Matchload Left Side
  combine.move(-40);
  intake.move(127);
  drive(18_in, 60);
   // Matchload
chassis.pid_drive_set(-1, 127);    
  pros::delay(50);

  chassis.pid_drive_set(4.5, 127);
  matchload(25);
  pros::delay(3000);
  matchload(0);
  intake.move(40);
  combine.move(0);

  // ===== Move to front Left Side =====
  drive(-12.25_in);
  collectorExtended = false;
  block_collector.set_value(collectorExtended);
  turn(180_deg);
  drive(14.5_in);
  hood.move(-127);
  intake.move(127);
  combine.move(-110);
  pros::delay(2500);
  drive(-15_in);
  turn(135_deg);
    combine.move(127);
   intake.move(-127); 
  drive(24_in);  
   drive(15_in);
   combine.move(127);
   intake.move(-127);
   pros::delay(2000);
   drive(-15_in);
   turn(180_deg);
   drive(68_in);
   turn(90_deg);
   drive(16_in);
   turn(180_deg);
   drive(60_in);
  pros::delay(2000);
}

  void Autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

    MatchAutonR(); // directly call your function

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}