#pragma once
#include "vex.h"

enum drive_setup: int {
  ZERO_TRACKER = 1, 
  TANK_ONE_ENCODER = 2, 
  TANK_ONE_ROTATION = 4, 
  TANK_TWO_ENCODER = 8, 
  TANK_TWO_ROTATION = 16, 
  HOLONOMIC_TWO_ENCODER = 32,
  HOLONOMIC_TWO_ROTATION = 64
};

class Drive
{
private:
  float wheel_diameter;
  float wheel_ratio;
  float gyro_scale;
  float drive_in_to_deg_ratio;
  float ForwardTracker_center_distance;
  float ForwardTracker_diameter;
  float ForwardTracker_in_to_deg_ratio;
  float SidewaysTracker_center_distance;
  float SidewaysTracker_diameter;
  float SidewaysTracker_in_to_deg_ratio;
  vex::triport ThreeWire = vex::triport(vex::PORT22);

public: 
  drive_setup drive_setup = ZERO_TRACKER;
  vex::motor_group DriveL;
  vex::motor_group DriveR;
  vex::inertial Gyro;
  vex::motor DriveLF;
  vex::motor DriveRF;
  vex::motor DriveLB;
  vex::motor DriveRB;
  vex::rotation R_ForwardTracker;
  vex::rotation R_SidewaysTracker;
  vex::encoder E_ForwardTracker;
  vex::encoder E_SidewaysTracker;

  float turn_max_voltage;
  float turn_kp;
  float turn_ki;
  float turn_kd;
  float turn_starti;

  float turn_settle_error;
  float turn_settle_time;
  float turn_timeout;

  float drive_max_voltage;
  float drive_kp;
  float drive_ki;
  float drive_kd;
  float drive_starti;
  
  float follow_mu;
  float follow_min_ld;
  float follow_max_ld;
  float follow_ld;

  float follow_max_acceleration;
  float follow_max_voltage;

  float follow_heading_error_turn_threshold;
  float follow_settle_error;
  float follow_settle_time;
  float follow_timeout;

  float drive_settle_error;
  float drive_settle_time;
  float drive_timeout;

  float drive_continue_error;
  float drive_continue_time;

  float heading_max_voltage;
  float heading_kp;
  float heading_ki;
  float heading_kd;
  float heading_starti;

  float swing_max_voltage;
  float swing_kp;
  float swing_ki;
  float swing_kd;
  float swing_starti;

  float swing_settle_error;
  float swing_settle_time;
  float swing_timeout;
  
  float desired_heading;

  bool stop_auton = false;

  Drive(enum::drive_setup drive_setup, vex::motor_group DriveL, vex::motor_group DriveR, int gyro_port, float wheel_diameter, float wheel_ratio, float gyro_scale, int DriveLF_port, int DriveRF_port, int DriveLB_port, int DriveRB_port, int ForwardTracker_port, float ForwardTracker_diameter, float ForwardTracker_center_distance, int SidewaysTracker_port, float SidewaysTracker_diameter, float SidewaysTracker_center_distance);

  void drive_with_voltage(float leftVoltage, float rightVoltage);
  void drive_keep_turn_rate(float leftVoltage, float rightVoltage);

  float get_absolute_heading();

  float get_left_position_in();

  float get_right_position_in();

  void set_turn_constants(float turn_max_voltage, float turn_kp, float turn_ki, float turn_kd, float turn_starti); 
  void set_drive_constants(float drive_max_voltage, float drive_kp, float drive_ki, float drive_kd, float drive_starti);
  void set_heading_constants(float heading_max_voltage, float heading_kp, float heading_ki, float heading_kd, float heading_starti);
  void set_swing_constants(float swing_max_voltage, float swing_kp, float swing_ki, float swing_kd, float swing_starti);
  void set_follow_constants(float follow_max_voltage, float follow_max_acceleration, float follow_max_applicable_curvature_radius, float follow_min_ld, float follow_max_ld, float follow_k_ld);

  void set_turn_exit_conditions(float turn_settle_error, float turn_settle_time, float turn_timeout);
  void set_drive_exit_conditions(float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_continue_error, float drive_continue_time);
  void set_swing_exit_conditions(float swing_settle_error, float swing_settle_time, float swing_timeout);
  void set_follow_exit_conditions(float follow_settle_error, float follow_settle_time, float follow_timeout);

  void set_brake_type(vex::brakeType mode);

  void turn_to_angle(float angle);
  void turn_to_angle(float angle, float turn_timeout);
  void turn_to_angle(float angle, float turn_timeout, float turn_max_voltage);
  void turn_to_angle(float angle, float turn_timeout, float turn_max_voltage, float turn_settle_error, float turn_settle_time);
  void turn_to_angle(float angle, float turn_timeout, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_kp, float turn_ki, float turn_kd, float turn_starti);

  void drive_time(float time, float voltage, bool stop = true);
  void drive_distance(float distance);
  void drive_distance(float distance, float heading);
  void drive_distance(float distance, float heading, float drive_timeout);
  void drive_distance(float distance, float heading, float drive_timeout, float drive_max_voltage, float heading_max_voltage);
  void drive_distance(float distance, float heading, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time);
  void drive_distance(float distance, float heading, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);
/*
  void follow(const std::string& path);
  void follow(const std::string& path, float heading_start_error);
  void follow(const std::string& path, float heading_start_error, float drive_max_voltage, float heading_max_voltage);
  void follow(const std::string& path, float heading_start_error, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout);
  void follow(const std::string& path, float heading_start_error, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);
  
  void follow(std::vector<PathPoint>& path, bool is_reverse=false);
  void follow(std::vector<PathPoint>& path, bool is_reverse, float min_ld, float max_ld, float k_ld);
  void follow(std::vector<PathPoint>& path, bool is_reverse, float min_ld, float max_ld, float k_ld, float follow_max_voltage, float heading_max_voltage, float follow_max_acceleration);
  void follow(std::vector<PathPoint>& path, bool is_reverse, float min_ld, float max_ld, float k_ld, float follow_max_voltage, float heading_max_voltage, float follow_max_acceleration, float heading_start_error, float follow_settle_error, float follow_settle_time, float follow_timeout);
  void follow(std::vector<PathPoint>& path, bool is_reverse, float min_ld, float max_ld, float k_ld, float follow_max_voltage, float heading_max_voltage, float follow_max_acceleration, float heading_start_error, float follow_settle_error, float follow_settle_time, float follow_timeout, float heading_kp, float heading_ki, float heading_kd, float heading_starti);
  */
  void stop(vex::brakeType mode);

  void left_swing_to_angle(float angle);
  void left_swing_to_angle(float angle, float swing_timeout);
  void left_swing_to_angle(float angle, float swing_timeout, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_kp, float swing_ki, float swing_kd, float swing_starti);
  void right_swing_to_angle(float angle);
  void right_swing_to_angle(float angle, float swing_timeout);
  void right_swing_to_angle(float angle, float drive_timeout, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_kp, float swing_ki, float swing_kd, float swing_starti);
  
  Odom odom;
  float get_ForwardTracker_position();
  float get_SidewaysTracker_position();
  void set_coordinates(float X_position, float Y_position, float orientation_deg);
  void position_track();
  static int position_track_task();
  vex::task async_task;
  float get_X_position();
  float get_Y_position();
  Vector2 get_position();

  void drive_to_point(float X_position, float Y_position, bool is_rigid=false, bool is_reverse=false);
  void drive_to_point(float X_position, float Y_position, bool is_rigid, bool is_reverse, float drive_timeout);
  void drive_to_point(float X_position, float Y_position, bool is_rigid, bool is_reverse, float drive_timeout, float drive_max_voltage, float heading_max_voltage);
  void drive_to_point(float X_position, float Y_position, bool is_rigid, bool is_reverse, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time);
  void drive_to_point(float X_position, float Y_position, bool is_rigid, bool is_reverse, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);

  void drive_through_points(const std::vector<Vector2>& path);
  void drive_through_points(const std::vector<Vector2>& path, float drive_timeout);
  void drive_through_points(const std::vector<Vector2>& path, float drive_timeout, float drive_max_voltage, float heading_max_voltage);
  void drive_through_points(const std::vector<Vector2>& path, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_continue_error, float drive_continue_time);
  void drive_through_points(const std::vector<Vector2>& path, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_continue_error, float drive_continue_time, float drive_settle_error, float drive_settle_time);
  void drive_through_points(const std::vector<Vector2>& path, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_continue_error, float drive_continue_time, float drive_settle_error, float drive_settle_time, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);

  void follow(const std::vector<Vector2>& path);
  void follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth = true);
  void follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth, float follow_max_voltage, float heading_max_voltage);
  void follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth, float follow_max_voltage, float heading_max_voltage, float ld);
  void follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth, float follow_max_voltage, float heading_max_voltage, float ld, float follow_settle_error, float follow_settle_time);
  void follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth, float follow_max_voltage, float heading_max_voltage, float ld, float follow_settle_error, float follow_settle_time, float follow_mu, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);
  
  void turn_to_point(float X_position, float Y_position);
  void turn_to_point(float X_position, float Y_position, float extra_angle_deg);
  void turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_timeout);
  void turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_timeout, float turn_max_voltage, float turn_settle_error, float turn_settle_time);
  void turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_timeout, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_kp, float turn_ki, float turn_kd, float turn_starti);
  
  void holonomic_drive_to_point(float X_position, float Y_position);
  void holonomic_drive_to_point(float X_position, float Y_position, float angle);
  void holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_timeout);
  void holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_timeout, float drive_max_voltage, float heading_max_voltage);
  void holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time);
  void holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);

  void control_arcade();
  void control_tank();
};