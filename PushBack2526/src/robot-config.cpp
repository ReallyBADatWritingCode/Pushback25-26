#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

controller Controller1 = controller(primary);
motor leftA = motor(PORT1, ratio18_1, false);
motor leftB = motor(PORT2, ratio18_1, false);
motor leftC = motor(PORT3, ratio18_1, false);
motor_group leftGroup = motor_group(leftA, leftB, leftC);
motor rightA = motor(PORT11, ratio18_1, true);
motor rightB = motor(PORT12, ratio18_1, true);
motor rightC = motor(PORT13, ratio18_1, true);
motor_group leftGroup = motor_group(rightA, rightB, rightC);
motor intake = motor(PORT10, ratio18_1, false);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}