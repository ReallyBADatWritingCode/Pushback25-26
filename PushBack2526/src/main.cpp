/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

bool intakeActive = false; 
bool lowerGoal = false;
bool middleGoal = false;
bool topGoal = false;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

void autonomous(void) {
  //Driving forward
  rightGroup.spinFor(forward, (24 / (10.2)), rotationUnits::rev, 50, percent, false);
  leftGroup.spinFor(forward, (24 / (10.2)), rotationUnits::rev, 50, percent, true);
  //Rotating towards triballs
  rightGroup.spinFor(reverse, (12 / 10.2), rotationUnits::rev, 50, percent, true);
  //Moving towards triballs
  rightGroup.spinFor(forward, (12 / (10.2)), rotationUnits::rev, 50, percent, false);
  leftGroup.spinFor(forward, (12 / (10.2)), rotationUnits::rev, 50, percent, true);
  //Intake Active
  intake.spin(forward, 100, percent);
  task::sleep(2000);
  intake.stop(coast);
  //Moving towards the matchloader
  leftGroup.spinFor(forward, (6 / (10.2)), rotationUnits::rev, 50, percent, true); //Rotating to face diagnaly
  //Moving diagnolly
  rightGroup.spinFor(forward, (34 / (10.2)), rotationUnits::rev, 50, percent, false); 
  leftGroup.spinFor(forward, (34 / (10.2)), rotationUnits::rev, 50, percent, true);
  //Rotating to matchloader
  rightGroup.spinFor(forward, (6 / (10.2)), rotationUnits::rev, 50, percent, true);
  //Pneumatic extend
  matchloaderExtend.set(true);
  matchloaderRetract.set(false);
  //Move Forward
  rightGroup.spinFor(forward, (24 / (10.2)), rotationUnits::rev, 100, percent, false);
  leftGroup.spinFor(forward, (24 / (10.2)), rotationUnits::rev, 100, percent, true);
  //Intake from matchloader
  intake.spin(forward, 100, percent);
  task::sleep(1000);
  intake.stop(coast);
  //Score
  rightGroup.spinFor(forward, (42 / (10.2)), rotationUnits::rev, 50, percent, false);
  leftGroup.spinFor(forward, (42 / (10.2)), rotationUnits::rev, 50, percent, true);
  topGoal.spin(reverse, 100, percent);
  task::sleep(2000);
  intake.stop(coast);
}

void Intake(){
  //Before if bool was true then it would be false and vice versa
  intakeActive = !intakeActive;
  //Intake active or not
  if(intakeActive){
    intake.spin(forward, 100, percent);
  }else intake.stop(coast); //Coast allows it to spin freely and naturally to stop.
}

void LowerGoal(){
  lowerGoal = !lowerGoal;
  if(lowerGoal) intake.spin(reverse, 100, percent);
  else intake.stop(coast);
}

void MiddleGoal(){
  middleGoal = !middleGoal;
  if(middleGoal){
    topOutTake.spin(forward, 100, percent);
    bottomOutTake.spin(reverse, 100, percent);
  }else outTake.stop(coast);
}

void TopGoal(){
  topGoal = !topGoal;
  if(topGoal) topOutTake.spin(reverse, 100, percent);
  else topOutTake.stop(coast);
}

void NonDriveMovement(){
  // Controls will be changed
  Controller1.ButtonA.pressed(Intake);
  Controller1.ButtonB.pressed(LowerGoal);
  Controller1.ButtonDown.pressed(MiddleGoal);
  Controller1.ButtonL1.pressed(TopGoal);
}

void TankDrive(){
  NonDriveMovement();
  while(true){
    leftGroup.spin(forward, (Controller1.Axis3.value()+Controller1.Axis1.value()), percent);
    rightGroup.spin(forward, (Controller1.Axis3.value()-Controller1.Axis1.value()), percent);
    wait(20, msec);
  }
}

void ArcadeDrive(){
  NonDriveMovement();
  while(true){
    leftGroup.spin(forward, Controller1.Axis3.value(), percent);
    rightGroup.spin(forward, Controller1.Axis2.value(), percent);
    wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(TankDrive); // Switch to arcade if wanted.

  pre_auton();
  while (true) {
    wait(20, msec);
  }
}
