#include "vex.h"
#include <iostream>

void default_constants(){
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, 0.4, 0, 1, 0);
  chassis.set_turn_constants(12, 0.39, 0.03, 3, 15);
  chassis.set_swing_constants(12, 0.4, 0.003, 2, 15);
  chassis.set_follow_constants(10, 0.24, 12, 6, 24, 14);
  chassis.set_drive_exit_conditions(1.5, 100, 3000, 5, 5);
  chassis.set_turn_exit_conditions(1.5, 100, 500);
  chassis.set_swing_exit_conditions(1, 100, 1000);
  chassis.set_follow_exit_conditions(1, 300, 6000);
}

void drive_test() {
  // pass
  chassis.set_coordinates(0, 0, 0);

  chassis.turn_to_point(48, 48);
  std::cout << '(' << chassis.get_X_position() << ", " << chassis.get_Y_position() << ", " << chassis.get_absolute_heading() << ')' << std::endl;
  chassis.drive_to_point(48, 48);
  chassis.drive_to_point(0, 0);
}

void turn_test(){
  chassis.turn_to_angle(5);
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(225);
  chassis.turn_to_angle(0);
}

void swing_test(){
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

void full_test(){
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

void odom_test(){
  //chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(0,50, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(0,70, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(0,90, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(0,110, "Left: %f", chassis.get_left_position_in());
    Brain.Screen.printAt(0,130, "Right: %f", chassis.get_right_position_in());
    std::cout << '(' << chassis.get_X_position() << ", " << chassis.get_Y_position() << ", " << chassis.get_absolute_heading() << ')' << std::endl;
    vex::task::sleep(20);
  }
}

void tank_odom_test(){
  chassis.set_coordinates(0, 0, 0);
  chassis.drive_to_point(6, 18);
  chassis.turn_to_point(12,0, 180);
  chassis.drive_to_point(12, 0);
  chassis.turn_to_angle(100);
  chassis.drive_to_point(0, 0);
}

void holonomic_odom_test(){
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_point(0, 18, 90);
  chassis.holonomic_drive_to_point(18, 0, 180);
  chassis.holonomic_drive_to_point(0, 18, 270);
  chassis.holonomic_drive_to_point(0, 0, 0);
}


