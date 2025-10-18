#include "vex.h"

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

void red_left() {
  /*
    chassis.set_coordinates(-128.349, 21.038, 0);
    leftGroup.setVelocity(60, percent);
    rightGroup.setVelocity(60, percent);
    //Collect 3 center goal triballs
    chassis.turn_to_angle(25);
    leftGroup.spin(forward);
    rightGroup.spin(forward);
    IntakeSystem.spin(forward);
    colorSensorMotor.spin(forward);
    task::sleep(1500);
    leftGroup.stop();
    rightGroup.stop();

    //center goal
    chassis.turn_to_angle(135);
    leftGroup.spin(reverse);
    rightGroup.spin(reverse);
    task::sleep(500);
    leftGroup.stop();
    rightGroup.stop();
    topOutake.spin(reverse);
    bottomOutTake.spin(forward);
    task::sleep(500);

    //Matchloader
    leftGroup.spin(forward);
    rightGroup.spin(forward);
    topOutake.stop();
    bottomOutTake.stop();
    task::sleep(500);
    leftGroup.stop();
    rightGroup.stop();
    chassis.turn_to_angle(180);
    task::sleep(250);
    Scraper.set(true);
    leftGroup.spin(forward);
    rightGroup.spin(forward);
    task::sleep(250);
    leftGroup.stop();
    rightGroup.stop();
    IntakeSystem.spin(forward);
    colorSensorMotor.spin(forward);
    task::sleep(500);
    
    //Top Goal
    IntakeSystem.stop();
    colorSensorMotor.stop();
    colorSensorMotor.stop();
    leftGroup.spin(reverse);
    rightGroup.spin(reverse);
    task::sleep(500);
    topOutake.spin(forward);
    bottomOutTake.spin(forward);
    leftGroup.stop();
    rightGroup.stop();
    task::sleep(500);
    */
}

void red_right() {
    chassis.set_coordinates(-128.349, -20.14, 0);
    leftGroup.setVelocity(60, percent);
    rightGroup.setVelocity(60, percent);
    IntakeSystem.setVelocity(100, percent);
    colorSensorMotor.setVelocity(100, percent);
    bottomOutTake.setVelocity(100, percent);
    colorSensorMotor.setVelocity(100, percent);
    //Collect first 3 triballs
    chassis.turn_to_angle(335);
    leftGroup.spin(forward);
    rightGroup.spin(forward);
    IntakeSystem.spin(forward);
    colorSensorMotor.spin(reverse);
    task::sleep(500);
    leftGroup.stop();
    rightGroup.stop();
    IntakeSystem.stop();
    colorSensorMotor.stop();
    //Center goal
    chassis.turn_to_angle(45);
    leftGroup.spin(reverse);
    rightGroup.spin(reverse);
    task::sleep(500);
    leftGroup.stop();
    rightGroup.stop();
    colorSensorMotor.spin(reverse);
    bottomOutTake.spin(forward);
    task::sleep(500);
    //Loader
    leftGroup.spin(forward);
    rightGroup.spin(forward);
    colorSensorMotor.stop();
    bottomOutTake.stop();
    task::sleep(500);
    leftGroup.stop();
    rightGroup.stop();
    chassis.turn_to_angle(180);
    task::sleep(250);
    Scraper.set(true);
    leftGroup.spin(forward);
    rightGroup.spin(forward);
    task::sleep(250);
    leftGroup.stop();
    rightGroup.stop();
    IntakeSystem.spin(forward);
    colorSensorMotor.spin(reverse);
    task::sleep(500);
    //Top goal
    Scraper.set(false);
    leftGroup.spin(reverse);
    rightGroup.spin(reverse);
    IntakeSystem.stop();
    colorSensorMotor.stop();
    task::sleep(500);
    colorSensorMotor.spin(forward);
    bottomOutTake.spin(forward);
}

void blue_left() {
    chassis.set_coordinates(140.447, -22.681, 205);
    leftGroup.setVelocity(60, percent);
    rightGroup.setVelocity(60, percent);
    IntakeSystem.setVelocity(100, percent);
    colorSensorMotor.setVelocity(100, percent);
    bottomOutTake.setVelocity(100, percent);
    colorSensorMotor.setVelocity(100, percent);
    //Collect first 3 triballs
    leftGroup.spin(forward);
    rightGroup.spin(forward);
    IntakeSystem.spin(forward);
    colorSensorMotor.spin(forward);
    task::sleep(700);
    leftGroup.stop();
    rightGroup.stop();
    IntakeSystem.stop();
    colorSensorMotor.stop();
    //Middle goal
    chassis.turn_to_angle(310);
    leftGroup.spin(reverse);
    rightGroup.spin(reverse);
    task::sleep(200);
    leftGroup.stop();
    rightGroup.stop();
    task::sleep(100);
    colorSensorMotor.spin(reverse);
    bottomOutTake.spin(forward);
    task::sleep(500);
    colorSensorMotor.stop();
    bottomOutTake.stop();
    //Matchloader
    leftGroup.spin(forward);
    rightGroup.spin(forward);
    task::sleep(1000);
    leftGroup.stop();
    rightGroup.stop();
    chassis.turn_to_angle(0);
    Scraper.set(true);
    IntakeSystem.spin(forward);
    colorSensorMotor.spin(reverse);
    leftGroup.spin(forward);
    rightGroup.spin(forward);
    task::sleep(800);
    leftGroup.stop();
    rightGroup.stop();
    IntakeSystem.stop();
    colorSensorMotor.stop();
    //Long goal
    leftGroup.spin(reverse);
    rightGroup.spin(reverse);
    task::sleep(600);
    leftGroup.stop();
    rightGroup.stop();
    colorSensorMotor.spin(forward);
    bottomOutTake.spin(forward);
    task::sleep(1000);
}

void blue_right() {
    chassis.set_coordinates(138.266, 21.481, 160);
    leftGroup.setVelocity(60, percent);
    rightGroup.setVelocity(60, percent);
    IntakeSystem.setVelocity(100, percent);
    colorSensorMotor.setVelocity(100, percent);
    bottomOutTake.setVelocity(100, percent);
    colorSensorMotor.setVelocity(100, percent);
    //Collect first 3 triballs
    leftGroup.spin(forward);
    rightGroup.spin(forward);
    IntakeSystem.spin(forward);
    task::sleep(700);
    leftGroup.stop();
    rightGroup.stop();
    IntakeSystem.stop();
    //Lower goal
    chassis.turn_to_angle(230);
    leftGroup.spin(reverse);
    rightGroup.spin(reverse);
    task::sleep(200);
    leftGroup.stop();
    rightGroup.stop();
    IntakeSystem.spin(forward);
    colorSensorMotor.spin(reverse);
    task::sleep(1000);
    IntakeSystem.stop();
    colorSensorMotor.stop();
    leftGroup.spin(forward);
    rightGroup.spin(forward);
    task::sleep(500);
    leftGroup.stop();
    rightGroup.stop();
    // Matchloader
    chassis.turn_to_angle(0);
    Scraper.set(true);
    IntakeSystem.spin(forward);
    colorSensorMotor.spin(reverse);
    leftGroup.spin(forward);
    rightGroup.spin(forward);
    task::sleep(300);
    leftGroup.stop();
    rightGroup.stop();
    task::sleep(1000);
    IntakeSystem.stop();
    colorSensorMotor.stop();
    //Top Goal
    Scraper.set(false);
    leftGroup.spin(reverse);
    rightGroup.spin(reverse);
    task::sleep(500);
    leftGroup.stop();
    rightGroup.stop();
    colorSensorMotor.spin(forward);
    bottomOutTake.spin(forward);
    task::sleep(1000);
}

void skills() {
    // skills auton code here
    return;
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


