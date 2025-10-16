#include "vex.h"
#include "robot-config.h"
#include <iostream>

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Inertial Senseor     inertial      6               

// LeftFront            motor         7 - 
// LeftMid              motor         10 -                    
// LeftBack             motor         8 -      
       
// RightFront           motor         20 -        
// RightMid             motor         19 -        
// RightBack            motor         11 -              
           
// Wings                digital_out   A               
// Scraper              digital_out   H               

// IntakeSystem         motor         16 -    
// MiddleConveyor       motor         15 -      
// OutakeSystem         motor         19     
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors using the V5 port icon in the top right of the screen. Doing     */
/*  so will update robot-config.cpp and robot-config.h automatically, so     */
  /*  you don't have to.                                                       */
  /*---------------------------------------------------------------------------*/

  /*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your robot manually with the sidebar configurer. */
/*---------------------------------------------------------------------------*/


Drive chassis(

//Specify your drive setup below. There are seven options:
//ZERO_TRACKER, TANK_ONE_ENCODER, TANK_ONE_ROTATION, TANK_TWO_ENCODER, TANK_TWO_ROTATION, HOLONOMIC_TWO_ENCODER, and HOLONOMIC_TWO_ROTATION
//TANK_ONE_ENCODER and TANK_ONE_ROTATION assumes that you have a sideways tracking wheel and will use right drive base motor encoders for forward tracking.
//For example, if you are not using odometry, put ZERO_TRACKER below:
ZERO_TRACKER,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(LeftFront, LeftMid, LeftBack),

//Right Motors:
motor_group(RightFront, RightMid, RightBack),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT6,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.75,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
-360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//PAUSE! The rest of the drive constructor is for robot using POSITION TRACKING.
//If you are not using position tracking, leave the rest of the values as they are.

//Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT1,

//LB:      //RB: 
PORT1,     -PORT1,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, leave it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1"
2,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
3.25,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//This distance is in inches:
0,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
5,

//Sideways tracker diameter (reverse to make the direction switch):
0,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
0

);

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/


enum auton_selection: int {
  // enter names of auton routines here
  AUTON_1
  
};

int current_auton_selection = AUTON_1;
bool auto_started = false;

int auton_selector_task() {
  while(auto_started == false) {
    Brain.Screen.clearScreen();
    switch(current_auton_selection){
      // copypaste 
      case AUTON_1:
        Brain.Screen.setPenColor(red);
        Brain.Screen.printAt(50, 50, "Auton 1");
        break;
    }

    if(Brain.Screen.pressing()) {
      while (Brain.Screen.pressing()) {}
      ++current_auton_selection;
    } else if (current_auton_selection == AUTON_1/*Last Auton in the enum*/){
      current_auton_selection = 0;
    }
    task::sleep(10);
  }
  return 0;
}

int subsystem_task() {
  while(true) {
    task::sleep(10);
  }
  return 0;
}

void autonomous(void) {
  switch(current_auton_selection) {  
    // add other auton routines here
    case AUTON_1:
      // call the name of the function
      // ex: BlueLeft();
      break;
  }
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();
  chassis.Gyro.calibrate();
  while (chassis.Gyro.isCalibrating()) wait(100, msec);
  //auto t = task(auton_selector_task);
  auto t2 = task(subsystem_task);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
    std::cout << "usercontrol started" << std::endl;
    while(chassis.Gyro.isCalibrating()) task::sleep(100); // wait for gyro calibration

    // bind any button press-release events here, i.e. Controller1.ButtonA.pressed(intake.toggle);

    // systems that are controlled by button holds and joysticks are done in the while loop below
    bool isRedTeam;
    bool chosen = false;
    while (true) {
        if(!chosen && Controller1.ButtonL2.PRESSED){
            chosen = true;
            isRedTeam = true;
        }else if(!chosen && Controller1.ButtonL1.PRESSED){
            chosen = true;
            isRedTeam = false;
        }
        // drivebase control
        chassis.set_brake_type(coast);
        // the numbers here are the joystick curve parameters.
        // first number is the "t" value, which controls how steep the curve is.
        // second number is the "alpha" value, which controls how sharply the curve transitions from shallow to steep.
        // third number is the deadband, which is how far the joystick has to be pushed before the input is registered.
        // fourth number is the angle range in which the joystick can be pushed such that the velocity will be determined by the distance to the center (see https://www.desmos.com/3d/qgptoofme0)
        // fifth number is the minimum output, which is the minimum speed the robot will move at when the joystick is pushed past the deadband.
        // speed ranges from -127 to +127.
        chassis.control_tank(); 
        //Intaking.
        if (Controller1.ButtonX.pressing())
        {
            colorSensor.setLightPower(100, percent);
            colorSensor.setLight(ledState::on);
            IntakeSystem.setVelocity(100, percent);
            IntakeSystem.spin(forward);
            double hue = colorSensor.hue();
            if((hue < 30) && (hue > 330)){
                colorSensorMotor.setVelocity(100, percent);
                (isRedTeam) ? colorSensorMotor.spin(forward) : colorSensorMotor.spin(reverse);
            }else if(hue > 180 && hue < 250){
                colorSensorMotor.setVelocity(100, percent);
                (isRedTeam) ? colorSensorMotor.spin(reverse) : colorSensorMotor.spin(forward);
            }
        }
        //LowerGoal
        else if (Controller1.ButtonY.pressing())
        {
            IntakeSystem.setVelocity(100, percent);
            IntakeSystem.spin(reverse);
        }
        //Middle goal
        else if (Controller1.ButtonR2.pressing())
        {
            IntakeSystem.setVelocity(100, percent);
            topOutake.setVelocity(100, percent);
            bottomOutTake.setVelocity(100, percent);
            topOutake.spin(reverse);
            bottomOutTake.spin(forward);
        }
        //Top Goal
        else if(Controller1.ButtonR1.pressing()){
            topOutake.setVelocity(100, percent);
            bottomOutTake.setVelocity(100, percent);
            topOutake.spin(forward);
            //Bottom outtakeis being used as middle conveyor here
            bottomOutTake.spin(forward);
        }
        else
        {
            IntakeSystem.stop();
            topOutake.stop();
            bottomOutTake.stop();
        } 
        if (Controller1.ButtonA.pressing())
        {
            Scraper.set(true);
        }
        else if(Controller1.ButtonB.pressing())
        {
            Scraper.set(false);
        }
        // input controls for other subsystems here
        task::sleep(10); // Sleep the task for a short amount of time to prevent wasted resources.
    }
}

//
// Main will set up the competition functions and callbacks. DO NOT MODIFY unless you know what you're doing.
//

int main() {
    // Set up callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
    
    pre_auton();

    // Prevent main from exiting using an infinite loop.
    while (true) {
        wait(100, msec);
    }
}
