#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
inertial InertialSensor = inertial(PORT6);
optical colorSensor(PORT14);

motor LeftFront = motor(PORT7, ratio6_1, true);
motor LeftMid = motor(PORT10, ratio6_1, true);
motor LeftBack = motor(PORT8, ratio6_1, true);
motor_group leftGroup = motor_group(LeftFront, LeftMid, LeftBack);


motor RightFront = motor(PORT20, ratio6_1, false);
motor RightMid = motor(PORT19, ratio6_1, false);
motor RightBack = motor(PORT11, ratio6_1, false);
motor_group rightGroup = motor_group(RightFront, RightMid, RightBack);

digital_out Scraper = digital_out(Brain.ThreeWirePort.A);

motor IntakeSystem = motor(PORT16, ratio6_1, false);
motor colorSensorMotor = motor(PORT15, ratio6_1, true);

motor topOutake = motor(PORT22, ratio6_1, true);
motor bottomOutTake = motor(PORT21, ratio6_1, true);



// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) 
{
  // nothing to initialize
}