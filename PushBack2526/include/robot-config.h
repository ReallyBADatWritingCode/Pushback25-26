using namespace vex;

extern brain Brain;

extern controller Controller1;

extern motor leftA;
extern motor leftB;
extern motor leftC;
extern motor_group leftGroup;
extern motor rightA;
extern motor rightB;
extern motor rightC;
extern motor_group rightGroup;
extern motor intake;
extern motor topOutTake;
extern motor bottomOutTake;
extern motor_group outTake;
extern pneumatics matchloaderExtend;
extern pneumatics matchloaderRetract;
extern optical colorSensor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
