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
bool gateClosed = true;
bool defenseActive = false;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

void Intake(){
  //Before if bool was true then it would be false and vice versa
  intakeActive = !intakeActive;
  //Intake active or not
  if(intakeActive){
    intake.spin(forward, 100, percent);
  }else intake.stop(coast); //Coast allows it to spin freely and naturally to stop.
}

void ClosingGate(){
  //Before if bool was true then it would be false and vice versa
  gateClosed = !gateClosed;
  backGatePneumatic.set(gateClosed);
}

void DefenseActivation(){
  //Before if bool was true then it would be false and vice versa
  defenseActive = !defenseActive;
  defensePneumatic.set(defenseActive);
}

void NonDriveMovement(){
  // Controls will be changed
  Controller1.ButtonA.pressed(Intake);
  Controller1.ButtonB.pressed(ClosingGate());
  Controller1.ButtonDown.pressed(DefenseActivation());
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
