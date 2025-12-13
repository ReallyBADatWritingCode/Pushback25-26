#include "vex.h"
using namespace vex;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
inertial InertialSensor = inertial(PORT1);

motor LeftFront = motor(PORT18, ratio6_1, true);
motor LeftMid = motor(PORT12, ratio6_1, true);
motor LeftBack = motor(PORT19, ratio6_1, true);
motor_group leftGroup = motor_group(LeftFront, LeftMid, LeftBack);


motor RightFront = motor(PORT6, ratio6_1, false);
motor RightMid = motor(PORT8, ratio6_1, false);
motor RightBack = motor(PORT10, ratio6_1, false);
motor_group rightGroup = motor_group(RightFront, RightMid, RightBack);

digital_out Scraper = digital_out(Brain.ThreeWirePort.B);
digital_out midGoal = digital_out(Brain.ThreeWirePort.A);

motor IntakeSystem = motor(PORT16, ratio6_1, false);
motor OutTakeSystem = motor(PORT17, ratio18_1, false);




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