#include "vex.h"

void default_constants()
{
  chassis.set_drive_constants(10, 2, 0, 0, 0);
  chassis.set_heading_constants(6, 1.2, 0, 1, 0); 
  chassis.set_turn_constants(12, .3, .3, .45, 0);
  chassis.set_swing_constants(12, 0.4, 0.003, 2, 15);
  chassis.set_follow_constants(10, 0.24, 12, 6, 24, 14);
  chassis.set_drive_exit_conditions(1.5, 150, 3000, 5, 5);
  //chassis.set_turn_exit_conditions(1.5, 150, 750);
  chassis.set_turn_exit_conditions(0.1, 500, 500);
  chassis.set_swing_exit_conditions(1, 100, 500);
  chassis.set_follow_exit_conditions(1, 300, 500);
}

void leftAuton()
{
  chassis.set_coordinates(-61, 17, -90);
  leftGroup.setVelocity(60, percent);
  rightGroup.setVelocity(60, percent);
  IntakeSystem.setVelocity(80, percent);
  OutTakeSystem.setVelocity(80, percent);
  // Getting 3 blocks at center goal
  chassis.drive_distance(21);
  chassis.turn_to_angle(-25);
  task::sleep(1000);
  chassis.drive_distance(10.4);
  IntakeSystem.spin(reverse);
  Scraper.set(true);
  task::sleep(3000);
  IntakeSystem.stop();
  Scraper.set(false);
  chassis.drive_distance(-7.2);
  chassis.turn_to_angle(-65);
  task::sleep(1000);
  chassis.drive_distance(32.44);
  //Long goal
  chassis.turn_to_angle(360);
  task::sleep(1000);
  chassis.drive_distance(-16);
  IntakeSystem.spin(reverse);
  OutTakeSystem.spin(reverse);
  task::sleep(3000);
  OutTakeSystem.stop();
  IntakeSystem.stop();
}

void rightAuton()
{
  chassis.set_coordinates(-61, -17, 0);
  leftGroup.setVelocity(60, percent);
  rightGroup.setVelocity(60, percent);
  IntakeSystem.setVelocity(80, percent);
  OutTakeSystem.setVelocity(80, percent);
  // Getting 3 blocks at center goal
  chassis.drive_distance(24.5);
  //chassis.turn_to_angle(-20);
  IntakeSystem.spin(reverse);
  chassis.drive_distance(14.4);
  //chassis.turn_to_angle(0);
  chassis.drive_distance(13);
  IntakeSystem.stop();
  //Long goal
  //chassis.turn_to_angle(-145);
  chassis.drive_distance(44);
  //chassis.turn_to_angle(-180);
  chassis.drive_distance(-16.5);
  OutTakeSystem.spin(reverse);
  IntakeSystem.spin(reverse);
  wait(1500, msec);
  OutTakeSystem.stop();
  IntakeSystem.stop();
  //Matchloader
  leftGroup.setVelocity(100, percent);
  rightGroup.setVelocity(100, percent);
  Scraper.set(true);
  IntakeSystem.spin(reverse);
  chassis.drive_distance(28);
  wait(2500, msec);
  IntakeSystem.stop();
  chassis.drive_distance(-28);
  IntakeSystem.spin(reverse);
  OutTakeSystem.spin(reverse);
  task::sleep(500);
}

void skills()
{
  //Start redLeft
}
void drive_test()
{
  // pass
  chassis.set_coordinates(0, 0, 0);

  chassis.turn_to_point(48, 48);
  std::cout << '(' << chassis.get_X_position() << ", " << chassis.get_Y_position() << ", " << chassis.get_absolute_heading() << ')' << std::endl;
  chassis.drive_to_point(48, 48);
  chassis.drive_to_point(0, 0);
}

void turn_test()
{
  chassis.turn_to_angle(5);
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(225);
  chassis.turn_to_angle(0);
}

void swing_test()
{
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

void full_test()
{
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

void odom_test()
{
  // chassis.set_coordinates(0, 0, 0);
  while (1)
  {
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(0, 50, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(0, 70, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(0, 90, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(0, 110, "Left: %f", chassis.get_left_position_in());
    Brain.Screen.printAt(0, 130, "Right: %f", chassis.get_right_position_in());
    std::cout << '(' << chassis.get_X_position() << ", " << chassis.get_Y_position() << ", " << chassis.get_absolute_heading() << ')' << std::endl;
    vex::task::sleep(20);
  }
}

void tank_odom_test()
{
  chassis.set_coordinates(0, 0, 0);
  chassis.drive_to_point(6, 18);
  chassis.turn_to_point(12, 0, 180);
  chassis.drive_to_point(12, 0);
  chassis.turn_to_angle(100);
  chassis.drive_to_point(0, 0);
}

void holonomic_odom_test()
{
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_point(0, 18, 90);
  chassis.holonomic_drive_to_point(18, 0, 180);
  chassis.holonomic_drive_to_point(0, 18, 270);
  chassis.holonomic_drive_to_point(0, 0, 0);
}
