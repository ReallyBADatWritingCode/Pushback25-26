#include "vex.h"
#include <iostream>
#include <vector>
#include <cmath>



Drive::Drive(enum::drive_setup drive_setup, vex::motor_group DriveL, vex::motor_group DriveR, int gyro_port, float wheel_diameter, float wheel_ratio, float gyro_scale, int DriveLF_port, int DriveRF_port, int DriveLB_port, int DriveRB_port, int ForwardTracker_port, float ForwardTracker_diameter, float ForwardTracker_center_distance, int SidewaysTracker_port, float SidewaysTracker_diameter, float SidewaysTracker_center_distance) :
  wheel_diameter(wheel_diameter),
  wheel_ratio(wheel_ratio),
  gyro_scale(gyro_scale),
  drive_in_to_deg_ratio(wheel_ratio/360.0*M_PI*wheel_diameter),
  ForwardTracker_center_distance(ForwardTracker_center_distance),
  ForwardTracker_diameter(ForwardTracker_diameter),
  ForwardTracker_in_to_deg_ratio(M_PI*ForwardTracker_diameter/360.0),
  SidewaysTracker_center_distance(SidewaysTracker_center_distance),
  SidewaysTracker_diameter(SidewaysTracker_diameter),
  SidewaysTracker_in_to_deg_ratio(M_PI*SidewaysTracker_diameter/360.0),
  drive_setup(drive_setup),
  DriveL(DriveL),
  DriveR(DriveR),
  Gyro(vex::inertial(gyro_port)),
  DriveLF(DriveLF_port, is_reversed(DriveLF_port)),
  DriveRF(DriveRF_port, is_reversed(DriveRF_port)),
  DriveLB(DriveLB_port, is_reversed(DriveLB_port)),
  DriveRB(DriveRB_port, is_reversed(DriveRB_port)),
  R_ForwardTracker(ForwardTracker_port),
  R_SidewaysTracker(SidewaysTracker_port),
  E_ForwardTracker(ThreeWire.Port[ForwardTracker_port-1]),
  E_SidewaysTracker(ThreeWire.Port[SidewaysTracker_port-1])
{
  if (drive_setup & ZERO_TRACKER)
    odom.set_physical_distances(0, 0);
  else if (drive_setup & (TANK_ONE_ENCODER | TANK_ONE_ROTATION))
    odom.set_physical_distances(0, SidewaysTracker_center_distance);
  else
    odom.set_physical_distances(ForwardTracker_center_distance, SidewaysTracker_center_distance);

  async_task = vex::task(position_track_task);
  set_coordinates(0,0,0);
}

void Drive::drive_with_voltage(float leftVoltage, float rightVoltage){
  DriveL.spin(vex::fwd, leftVoltage, vex::volt);
  DriveR.spin(vex::fwd, rightVoltage, vex::volt);
}

void Drive::drive_keep_turn_rate(float leftVoltage, float rightVoltage) {
  auto high = fmaxf(fabsf(leftVoltage), fabsf(rightVoltage));
  if (high > 12.0f) {
    high = 1.0f / high;
    leftVoltage *= high;
    rightVoltage *= high;
  }
  DriveL.spin(vex::fwd, leftVoltage, vex::volt);
  DriveR.spin(vex::fwd, rightVoltage, vex::volt);
}

void Drive::set_turn_constants(float turn_max_voltage, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  this->turn_max_voltage = turn_max_voltage;
  this->turn_kp = turn_kp;
  this->turn_ki = turn_ki;
  this->turn_kd = turn_kd;
  this->turn_starti = turn_starti;
} 

void Drive::set_drive_constants(float drive_max_voltage, float drive_kp, float drive_ki, float drive_kd, float drive_starti){
  this->drive_max_voltage = drive_max_voltage;
  this->drive_kp = drive_kp;
  this->drive_ki = drive_ki;
  this->drive_kd = drive_kd;
  this->drive_starti = drive_starti;
} 

void Drive::set_heading_constants(float heading_max_voltage, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  this->heading_max_voltage = heading_max_voltage;
  this->heading_kp = heading_kp;
  this->heading_ki = heading_ki;
  this->heading_kd = heading_kd;
  this->heading_starti = heading_starti;
}

void Drive::set_swing_constants(float swing_max_voltage, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  this->swing_max_voltage = swing_max_voltage;
  this->swing_kp = swing_kp;
  this->swing_ki = swing_ki;
  this->swing_kd = swing_kd;
  this->swing_starti = swing_starti;
} 

void Drive::set_follow_constants(float follow_max_voltage, float follow_max_acceleration, float follow_max_applicable_curvature_radius, float follow_min_ld, float follow_max_ld, float follow_ld) {
  this->follow_max_voltage = follow_max_voltage;
  this->follow_max_acceleration = follow_max_acceleration;
  this->follow_min_ld = follow_min_ld;
  this->follow_max_ld = follow_max_ld;
  this->follow_ld = follow_ld;
  this->follow_mu = 12.0f / sqrtf(follow_max_applicable_curvature_radius);
}

void Drive::set_turn_exit_conditions(float turn_settle_error, float turn_settle_time, float turn_timeout){
  this->turn_settle_error = turn_settle_error;
  this->turn_settle_time = turn_settle_time;
  this->turn_timeout = turn_timeout;
}

void Drive::set_drive_exit_conditions(float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_continue_error, float drive_continue_time){
  this->drive_settle_error = drive_settle_error;
  this->drive_settle_time = drive_settle_time;
  this->drive_timeout = drive_timeout;
  this->drive_continue_error = drive_continue_error;
  this->drive_continue_time = drive_continue_time;
}

void Drive::set_swing_exit_conditions(float swing_settle_error, float swing_settle_time, float swing_timeout){
  this->swing_settle_error = swing_settle_error;
  this->swing_settle_time = swing_settle_time;
  this->swing_timeout = swing_timeout;
}

void Drive::set_follow_exit_conditions(float follow_settle_error, float follow_settle_time, float follow_timeout) {
  this->follow_settle_error = follow_settle_error;
  this->follow_timeout = follow_timeout;
  this->follow_settle_time = follow_settle_time;
  this->follow_heading_error_turn_threshold = follow_heading_error_turn_threshold;
}

void Drive::set_brake_type(vex::brakeType mode) {
  this->DriveL.setStopping(mode);
  this->DriveR.setStopping(mode);
}

float Drive::get_absolute_heading(){ 
  return( reduce_0_to_360( Gyro.rotation()*360.0/gyro_scale ) ); 
}

float Drive::get_left_position_in(){
  return( DriveL.position(vex::deg) * drive_in_to_deg_ratio );
}

float Drive::get_right_position_in(){
  return( DriveR.position(vex::deg) * drive_in_to_deg_ratio );
}

void Drive::turn_to_angle(float angle){
  turn_to_angle(angle, turn_timeout, turn_max_voltage, turn_settle_error, turn_settle_time, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_timeout){
  turn_to_angle(angle, turn_timeout, turn_max_voltage, turn_settle_error, turn_settle_time, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_timeout, float turn_max_voltage){
  turn_to_angle(angle, turn_timeout, turn_max_voltage, turn_settle_error, turn_settle_time, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_timeout, float turn_max_voltage, float turn_settle_error, float turn_settle_time){
  turn_to_angle(angle, turn_timeout, turn_max_voltage, turn_settle_error, turn_settle_time, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_timeout, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  if (this->stop_auton) return;
  desired_heading = angle;
  PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while (!this->stop_auton && turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(-output, output);
    std::cout << get_absolute_heading() << std::endl;
    vex::task::sleep(10);
  }
  if (!this->stop_auton) {
    DriveL.stop(vex::hold);
    DriveR.stop(vex::hold);
  } else {
    DriveR.setStopping(vex::coast);
    DriveL.setStopping(vex::coast);
  }
}

void Drive::drive_time(float time, float voltage, bool stop) {
  //this->DriveL.setVelocity(voltage, vex::volt);
  //this->DriveR.setVelocity(voltage, vex::volt);
  this->DriveL.spin(vex::fwd, voltage, vex::volt);
  this->DriveR.spin(vex::fwd, voltage, vex::volt);
  wait(time, vex::msec);
  if (stop) {
    this->stop(vex::brake);
  }
}

void Drive::stop(vex::brakeType mode) {
  this->DriveL.stop(mode);
  this->DriveR.stop(mode);
}

void Drive::drive_distance(float distance){
  drive_distance(distance, desired_heading, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading){
  drive_distance(distance, heading, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_timeout){
  drive_distance(distance, heading, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_timeout, float drive_max_voltage, float heading_max_voltage) {
  drive_distance(distance, heading, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time){
  drive_distance(distance, heading, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  desired_heading = heading;
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;
  while(!this->stop_auton && drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage(drive_output-heading_output, drive_output+heading_output);
    vex::task::sleep(10);
  }
  if (!this->stop_auton) {
    DriveL.stop(vex::hold);
    DriveR.stop(vex::hold);
  } else {
    DriveR.setStopping(vex::coast);
    DriveL.setStopping(vex::coast);
  }
}

void Drive::left_swing_to_angle(float angle){
  left_swing_to_angle(angle, swing_timeout, swing_max_voltage, swing_settle_error, swing_settle_time, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::left_swing_to_angle(float angle, float swing_timeout){
  left_swing_to_angle(angle, swing_timeout, swing_max_voltage, swing_settle_error, swing_settle_time, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::left_swing_to_angle(float angle, float swing_timeout, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  if (this->stop_auton) return;
  desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(!this->stop_auton && swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    DriveL.spin(vex::fwd, -output, vex::volt);
    DriveR.stop(vex::hold);
    vex::task::sleep(10);
  }
  if (!this->stop_auton) {
    DriveL.stop(vex::hold);
    DriveR.stop(vex::hold);
  } else {
    DriveR.setStopping(vex::coast);
    DriveL.setStopping(vex::coast);
  }
}

void Drive::right_swing_to_angle(float angle){
  right_swing_to_angle(angle, swing_timeout, swing_max_voltage, swing_settle_error, swing_settle_time, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::right_swing_to_angle(float angle, float swing_timeout){
  right_swing_to_angle(angle, swing_timeout, swing_max_voltage, swing_settle_error, swing_settle_time, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::right_swing_to_angle(float angle, float swing_timeout, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  if (this->stop_auton) return;
  desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(!this->stop_auton && swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    DriveR.spin(vex::reverse, -output, vex::volt);
    DriveL.stop(vex::hold);
    vex::task::sleep(10);
  }
  if (!this->stop_auton) {
    DriveL.stop(vex::hold);
    DriveR.stop(vex::hold);
  } else {
    DriveL.setStopping(vex::coast);
    DriveR.setStopping(vex::coast);
  }
} 
/*
void Drive::follow(std::vector<PathPoint>& path, bool is_reverse) {
  follow(path, is_reverse, follow_min_ld, follow_max_ld, follow_ld, follow_max_voltage, heading_max_voltage, follow_max_acceleration, follow_heading_error_turn_threshold, follow_settle_error, follow_settle_time, follow_timeout, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow(std::vector<PathPoint>& path, bool is_reverse, float min_ld, float max_ld, float ld) {
    follow(path, is_reverse, follow_min_ld, follow_max_ld, follow_ld, follow_max_voltage, heading_max_voltage, follow_max_acceleration, follow_heading_error_turn_threshold, follow_settle_error, follow_settle_time, follow_timeout, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow(std::vector<PathPoint>& path, bool is_reverse, float min_ld, float max_ld, float ld, float follow_max_voltage, float heading_max_voltage, float follow_max_acceleration) {
    follow(path, is_reverse, follow_min_ld, follow_max_ld, follow_ld, follow_max_voltage, heading_max_voltage, follow_max_acceleration, follow_heading_error_turn_threshold, follow_settle_error, follow_settle_time, follow_timeout, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow(std::vector<PathPoint>& path, bool is_reverse, float min_ld, float max_ld, float ld, float follow_max_voltage, float heading_max_voltage, float follow_max_acceleration, float follow_heading_error_turn_threshold, float follow_settle_error, float follow_settle_time, float follow_timeout) {
    follow(path, is_reverse, follow_min_ld, follow_max_ld, follow_ld, follow_max_voltage, heading_max_voltage, follow_max_acceleration, follow_heading_error_turn_threshold, follow_settle_error, follow_settle_time, follow_timeout, heading_kp, heading_ki, heading_kd, heading_starti);
}
  
void Drive::follow(std::vector<PathPoint>& path, bool is_reverse, float min_ld, float max_ld, float ld, float follow_max_voltage, float heading_max_voltage, float follow_max_acceleration, float follow_heading_start_error, float follow_settle_error, float follow_settle_time, float follow_timeout, float heading_kp, float heading_ki, float heading_kd, float heading_starti) {
  if (path.empty()) return;

  float heading_offset = is_reverse ? 180.0f : 0.0f;

  auto pure_pursuit = PurePursuit(path, min_ld, max_ld, ld, follow_max_acceleration, follow_settle_error, follow_settle_time, follow_timeout);
  
  Vector2 pos = Vector2(this->get_X_position(), this->get_Y_position());
  pure_pursuit.compute(pos);

  // face first point if necessary 
  auto target_heading = pure_pursuit.get_target_heading(pos) + heading_offset;
  auto heading_error = reduce_negative_180_to_180(target_heading - get_absolute_heading());

  if (fabsf(heading_error) > follow_heading_start_error) {
    // face approximately the correct direction
    turn_to_angle(target_heading, turn_max_voltage, follow_heading_start_error, 250, 1000);
  }

  PID headingPID(heading_error, heading_kp, heading_ki, heading_kd, heading_starti);

  while (!pure_pursuit.is_settled()) {
    pos.x = this->get_X_position();
    pos.y = this->get_Y_position();

    pure_pursuit.compute(pos);

    heading_error = reduce_negative_180_to_180(pure_pursuit.get_target_heading(pos) + heading_offset - get_absolute_heading());
    float heading_scale_factor = cos(to_rad(heading_error));
    float scaled_drive_max_voltage = fabs(heading_scale_factor) * follow_max_voltage;

    float heading_output = clamp(headingPID.compute(heading_error), -heading_max_voltage, heading_max_voltage);
    float drive_output = clamp(pure_pursuit.get_target_speed() * heading_scale_factor, -scaled_drive_max_voltage, scaled_drive_max_voltage);

    drive_with_voltage(drive_output - heading_output, drive_output + heading_output);
    vex::task::sleep(10);
  }
}*/

float Drive::get_ForwardTracker_position(){
  if (drive_setup & (TANK_TWO_ENCODER | HOLONOMIC_TWO_ENCODER))
    return(E_ForwardTracker.position(vex::deg) * ForwardTracker_in_to_deg_ratio);
  if (drive_setup & (TANK_ONE_ENCODER | TANK_ONE_ROTATION | ZERO_TRACKER))
    return ( get_right_position_in() + get_left_position_in() ) * 0.5;
  return(R_ForwardTracker.position(vex::deg) * ForwardTracker_in_to_deg_ratio);
}

float Drive::get_SidewaysTracker_position(){
  if (drive_setup & (TANK_ONE_ENCODER | TANK_ONE_ROTATION | ZERO_TRACKER))
    return 0;
  if (drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER)
    return(E_SidewaysTracker.position(vex::deg) * SidewaysTracker_in_to_deg_ratio);
  return(R_SidewaysTracker.position(vex::deg) * SidewaysTracker_in_to_deg_ratio);
}

void Drive::position_track(){
  while(true) {
    odom.update_position(get_ForwardTracker_position(), get_SidewaysTracker_position(), get_absolute_heading());
    vex::task::sleep(5);
  }
}

void Drive::set_coordinates(float X_position, float Y_position, float orientation_deg){
  this->Gyro.setRotation(orientation_deg / gyro_scale * 360, vex::deg);
  this->desired_heading = orientation_deg;
  odom.set_position(X_position, Y_position, orientation_deg, get_ForwardTracker_position(), get_SidewaysTracker_position());
}

int Drive::position_track_task(){
  chassis.position_track();
  return(0);
}

float Drive::get_X_position(){
  return(odom.X_position);
}

float Drive::get_Y_position(){
  return(odom.Y_position);
}

Vector2 Drive::get_position() {
  return Vector2(odom.X_position, odom.Y_position);
}

void Drive::drive_to_point(float X_position, float Y_position, bool is_rigid, bool is_reverse){
  drive_to_point(X_position, Y_position, is_rigid, is_reverse, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, bool is_rigid, bool is_reverse, float drive_timeout){
  drive_to_point(X_position, Y_position, is_rigid, is_reverse, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, bool is_rigid, bool is_reverse, float drive_timeout, float drive_max_voltage, float heading_max_voltage){
  drive_to_point(X_position, Y_position, is_rigid, is_reverse, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, bool is_rigid, bool is_reverse, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time){
  drive_to_point(X_position, Y_position, is_rigid, is_reverse, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, bool is_rigid, bool is_reverse, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  if (this->stop_auton) return;

  if (is_rigid) {
    this->turn_to_point(X_position, Y_position, is_reverse ? 180.0f : 0.0f);
  }

  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(to_deg(atan2(Y_position-get_Y_position(), X_position - get_X_position()))-get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  while(!this->stop_auton && drivePID.is_settled() == false){
    float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(Y_position - get_Y_position(), X_position - get_X_position())) - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output *= heading_scale_factor;
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);
    
    if (drive_error<drive_settle_error) { heading_output = 0; }

    float scaled_drive_max_voltage = fabs(heading_scale_factor)*drive_max_voltage;
    drive_output = clamp(drive_output, -scaled_drive_max_voltage, scaled_drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage(drive_output-heading_output, drive_output+heading_output);
    vex::task::sleep(10);
  }
  if (!this->stop_auton) {
    DriveL.stop(vex::hold);
    DriveR.stop(vex::hold);
  } else {
    DriveR.setStopping(vex::coast);
    DriveL.setStopping(vex::coast);
  }
}

void Drive::drive_through_points(const std::vector<Vector2> &path) {
  drive_through_points(path, drive_timeout, drive_max_voltage, heading_max_voltage, drive_continue_error, drive_continue_time, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_through_points(const std::vector<Vector2> &path, float drive_timeout) {
  drive_through_points(path, drive_timeout, drive_max_voltage, heading_max_voltage, drive_continue_error, drive_continue_time, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_through_points(const std::vector<Vector2> &path, float drive_timeout, float drive_max_voltage, float heading_max_voltage) {
  drive_through_points(path, drive_timeout, drive_max_voltage, heading_max_voltage, drive_continue_error, drive_continue_time, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_through_points(const std::vector<Vector2> &path, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_continue_error, float drive_continue_time) {
  drive_through_points(path, drive_timeout, drive_max_voltage, heading_max_voltage, drive_continue_error, drive_continue_time, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_through_points(const std::vector<Vector2> &path, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_continue_error, float drive_continue_time, float drive_settle_error, float drive_settle_time) {
  drive_through_points(path, drive_timeout, drive_max_voltage, heading_max_voltage, drive_continue_error, drive_continue_time, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_through_points(const std::vector<Vector2>& path, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_continue_error, float drive_continue_time, float drive_settle_error, float drive_settle_time, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti) {
  if (this->stop_auton || path.empty()) return;

  PID headingPID(reduce_negative_180_to_180(to_deg((path[0] - get_position()).angle()) - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  // compute distances
  float distance_remaining = 0;
  std::vector<float> distances;
  for (int i = 0; i < path.size() - 1; ++i) {
    distances.emplace_back((path[i] - path[i+1]).norm());
    distance_remaining += distances.back();
  }
  distances.emplace_back(0);

  for (int i = 0; i < path.size() - 1; ++i) {
    auto target_point = path[i];
    auto next_target_point = path[i+1];
    PID drivePID((target_point - get_position()).norm(), drive_kp, drive_ki, drive_kd, drive_starti, drive_continue_error, drive_continue_time, drive_timeout);

    while(!this->stop_auton && drivePID.time_spent_settled < 15) {
      float drive_error = (target_point - get_position()).norm();
      float heading_error = reduce_negative_180_to_180(to_deg((target_point - get_position()).angle()) - get_absolute_heading());
      float added_speed = 0;

      if (drive_error < 12) {
        float next_heading_error = reduce_negative_180_to_180(to_deg((next_target_point - target_point).angle()) - get_absolute_heading());
        float next_drive_scale_factor = cosf(next_heading_error);
        added_speed = drive_kp / sqrtf(drive_error) * fabsf(next_drive_scale_factor * distances[i]); // rough approximation of the speed coming out
        heading_error = lerp<float>(next_heading_error, heading_error, drive_error / 12);
      }
      float drive_output = drivePID.compute(drive_error) + added_speed;

      float heading_scale_factor = cos(to_rad(heading_error));
      drive_output *= heading_scale_factor;
      heading_error = reduce_negative_90_to_90(heading_error);
      float heading_output = headingPID.compute(heading_error);
        
      if (drive_error<drive_settle_error) { heading_output = 0; }

      float scaled_drive_max_voltage = fabs(heading_scale_factor)*drive_max_voltage;
      drive_output = clamp(drive_output, -scaled_drive_max_voltage, scaled_drive_max_voltage);
      heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

      drive_with_voltage(drive_output-heading_output, drive_output+heading_output);
      vex::task::sleep(10);
    }
  }
  drive_to_point(path.back().x, path.back().y, false, false, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow(const std::vector<Vector2>& path) {
  follow(path, follow_timeout, true, follow_max_voltage, heading_max_voltage, follow_ld, follow_settle_error, follow_settle_time, follow_mu, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth) {
  follow(path, follow_timeout, make_smooth, follow_max_voltage, heading_max_voltage, follow_ld, follow_settle_error, follow_settle_time, follow_mu, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth, float follow_max_voltage, float heading_max_voltage) {
  follow(path, follow_timeout, make_smooth, follow_max_voltage, heading_max_voltage, follow_ld, follow_settle_error, follow_settle_time, follow_mu, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}
void Drive::follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth, float follow_max_voltage, float heading_max_voltage, float ld) {
  follow(path, follow_timeout, make_smooth, follow_max_voltage, heading_max_voltage, follow_ld, follow_settle_error, follow_settle_time, follow_mu, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}
void Drive::follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth, float follow_max_voltage, float heading_max_voltage, float ld, float follow_settle_error, float follow_settle_time) {
  follow(path, follow_timeout, make_smooth, follow_max_voltage, heading_max_voltage, follow_ld, follow_settle_error, follow_settle_time, follow_mu, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth, float follow_max_voltage, float heading_max_voltage, float ld, float follow_settle_error, float follow_settle_time, float follow_mu, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti) {
  if (path.empty() || this->stop_auton) return;
  if (path.size() == 1) {
    drive_to_point(path[0].x, path[0].y, false, false, drive_timeout, drive_max_voltage, heading_max_voltage, follow_settle_error, follow_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
    return;
  }

  std::vector<Vector2> waypoints;
  int max_waypoint_index;
  
  if (make_smooth) {
    // fill waypoints
    for (int i = 0; i < path.size() - 1; ++i) {
      auto direction_vector = (path[i+1] - path[i]).unit_vector();
      auto point = path[i];
      int length = (int)(path[i+1] - path[i]).norm();
      for (int j = 0; j <= length; ++j) {
        waypoints.emplace_back(point);
        point += direction_vector;
        if ((point - path[i]).norm_squared() < 0.25) break;
      }
    }

    max_waypoint_index = waypoints.size() - 1;

    // smoothing
    const float min_squared_deviation_to_keep_smoothing = 0.001;
    if (waypoints.size() >= 3) {
      const float t = 0.9;  // smoothing weight

      float max_squared_deviation;
      do {
        max_squared_deviation = 0;
        auto reference = waypoints;

        for (int i = 1; i < max_waypoint_index; ++i) {
          waypoints[i] = lerp<Vector2>(reference[i], (reference[i-1] + reference[i+1]) * 0.5, t);
          max_squared_deviation = std::fmaxf((waypoints[i] - reference[i]).norm_squared(), max_squared_deviation);
        }
      } while (max_squared_deviation >= min_squared_deviation_to_keep_smoothing);
    }
  } else {
    waypoints = path;
    max_waypoint_index = waypoints.size() - 1;
  }

  // compute distances
  float distance_remaining = 0;
  std::vector<float> distances;
  for (int i = 0; i < max_waypoint_index; ++i) {
    distances.emplace_back((waypoints[i+1] - waypoints[i]).norm());
    distance_remaining += distances.back();
  }
  distances.emplace_back(0);

  // get initial look ahead point
  int look_ahead_index = 0;
  int nearest_index = 0;
  Vector2 look_ahead_point = waypoints.front();
  
  while (look_ahead_index < max_waypoint_index && (look_ahead_point - get_position()).norm() < ld) {
    distance_remaining -= distances[look_ahead_index++];
    look_ahead_point = waypoints[look_ahead_index];
  }

  // get initial nearest point
  float squared_distance_to_path;
  float next_squared_distance;
  
  while (nearest_index < max_waypoint_index) {
    squared_distance_to_path = (path[nearest_index] - get_position()).norm_squared();
    next_squared_distance = (path[nearest_index + 1] - get_position()).norm_squared();
    if (next_squared_distance < squared_distance_to_path) ++nearest_index;
    else break;
  }

  // follow path
  PID drivePID(distance_remaining + (look_ahead_point - get_position()).norm(), drive_kp, drive_ki, drive_kd, drive_starti, follow_settle_error, follow_settle_time, follow_timeout);
  PID headingPID(reduce_negative_180_to_180(to_deg((look_ahead_point - get_position()).angle()) - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  
  while (!this->stop_auton && !drivePID.is_settled()) {
    // update look ahead point
    while (look_ahead_index < max_waypoint_index && (look_ahead_point - get_position()).norm() < follow_ld) {
      distance_remaining -= distances[look_ahead_index++];
      look_ahead_point = waypoints[look_ahead_index];
    }
  
    // update nearest point
    while (nearest_index < max_waypoint_index) {
      squared_distance_to_path = (path[nearest_index] - get_position()).norm_squared();
      next_squared_distance = (path[nearest_index + 1] - get_position()).norm_squared();
      if (next_squared_distance < squared_distance_to_path) ++nearest_index;
      else break;
    }
    
    // pursue look ahead point
    float heading_error = reduce_negative_180_to_180(to_deg((look_ahead_point - get_position()).angle()) - get_absolute_heading());
    float heading_scale_factor = cos(to_rad(heading_error));
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    float drive_error = distance_remaining + (look_ahead_point - get_position()).norm();
    float drive_output = drivePID.compute(drive_error); // update PID so that the derivative and integral is accurate.

    if (nearest_index != 0 && look_ahead_index != max_waypoint_index) {
      // Use v = sqrt(µgR), µ = coefficient of friction, g = gravity, and R = radius of curvature. Note that follow_mu = sqrt(µg).
      if (!(waypoints[nearest_index] - waypoints[nearest_index - 1]).is_parallel(waypoints[nearest_index + 1] - waypoints[nearest_index])) {
        float radius_of_curvature = three_point_circle_radius(waypoints[nearest_index - 1], waypoints[nearest_index], waypoints[nearest_index + 1]);
        drive_output = std::fminf(drive_output, follow_mu * sqrtf(radius_of_curvature));
      } else {
        drive_output = std::fminf(drive_output, follow_max_voltage);
      }
    }
    drive_output = clamp(drive_output, -follow_max_voltage, follow_max_voltage);
    drive_output *= heading_scale_factor;

    if (drive_error < drive_settle_error) { heading_output = 0; }

    drive_with_voltage(drive_output - heading_output, drive_output + heading_output);
    vex::task::sleep(10);
  }
  if (!this->stop_auton) {
    DriveL.stop(vex::hold);
    DriveR.stop(vex::hold);
  } else {
    DriveR.setStopping(vex::coast);
    DriveL.setStopping(vex::coast);
  }
}


void Drive::turn_to_point(float X_position, float Y_position){
  turn_to_point(X_position, Y_position, 0, turn_timeout, turn_max_voltage, turn_settle_error, turn_settle_time, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_timeout, turn_max_voltage, turn_settle_error, turn_settle_time, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_timeout){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_timeout, turn_max_voltage, turn_settle_error, turn_settle_time, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_timeout, float turn_max_voltage, float turn_settle_error, float turn_settle_time){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_timeout, turn_max_voltage, turn_settle_error, turn_settle_time, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_timeout, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  if (this->stop_auton) return;
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(Y_position-get_Y_position(), X_position - get_X_position())) - get_absolute_heading() + extra_angle_deg), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(!this->stop_auton && turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(to_deg(atan2(Y_position-get_Y_position(), X_position - get_X_position())) - get_absolute_heading() + extra_angle_deg);
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(-output, output);
    vex::task::sleep(10);
  }
  if (!this->stop_auton) {
    DriveL.stop(vex::hold);
    DriveR.stop(vex::hold);
  } else {
    DriveR.setStopping(vex::coast);
    DriveL.setStopping(vex::coast);
  }
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position){
  holonomic_drive_to_point(X_position, Y_position, get_absolute_heading(), drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_timeout){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_timeout, float drive_max_voltage, float heading_max_voltage){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  if (this->stop_auton) return;
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(Y_position-get_Y_position(), X_position-get_X_position()))-get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  while(!this->stop_auton && !( drivePID.is_settled() && turnPID.is_settled() ) ){
    float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    float turn_error = reduce_negative_180_to_180(to_deg(atan2(Y_position-get_Y_position(), X_position-get_X_position()))-get_absolute_heading());

    float drive_output = drivePID.compute(drive_error);
    float turn_output = turnPID.compute(turn_error);

    drive_output = clamp(drive_output, drive_max_voltage, drive_max_voltage);
    turn_output = clamp(turn_output, -heading_max_voltage, heading_max_voltage);

    float heading_error = atan2(Y_position-get_Y_position(), X_position-get_X_position());

    DriveLF.spin(vex::fwd, drive_output*cos(to_rad(get_absolute_heading()) + heading_error - M_PI/4) + turn_output, vex::volt);
    DriveLB.spin(vex::fwd, drive_output*cos(-to_rad(get_absolute_heading()) - heading_error + 3*M_PI/4) + turn_output, vex::volt);
    DriveRB.spin(vex::fwd, drive_output*cos(to_rad(get_absolute_heading()) + heading_error - M_PI/4) - turn_output, vex::volt);
    DriveRF.spin(vex::fwd, drive_output*cos(-to_rad(get_absolute_heading()) - heading_error + 3*M_PI/4) - turn_output, vex::volt);
    vex::task::sleep(10);
  }
  DriveLF.stop(vex::hold);
  DriveLB.stop(vex::hold);
  DriveRB.stop(vex::hold);
  DriveRF.stop(vex::hold);
}

void Drive::control_arcade(){
  auto velCtrl = map_sensitivity(vex::controller(vex::primary).Axis3.value());
  auto turnCtrl = map_sensitivity(vex::controller(vex::primary).Axis1.value(), 1.5);
  DriveL.spin(vex::fwd, to_volt(velCtrl+turnCtrl), vex::volt);
  DriveR.spin(vex::fwd, to_volt(velCtrl-turnCtrl), vex::volt);
}

void Drive::control_tank(){
  DriveL.spin(vex::fwd, to_volt(map_sensitivity(vex::controller(vex::primary).Axis3.value())), vex::volt);
  DriveR.spin(vex::fwd, to_volt(map_sensitivity(vex::controller(vex::primary).Axis2.value())), vex::volt);
}