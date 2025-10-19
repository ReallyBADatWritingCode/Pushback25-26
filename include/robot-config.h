using namespace vex;

extern brain  Brain;

extern controller Controller1;
extern inertial InertialSensor;
extern optical colorSensor;

extern motor LeftFront;
extern motor LeftMid;
extern motor LeftBack;
extern motor_group leftGroup;


extern motor RightFront;
extern motor RightMid;
extern motor RightBack;
extern motor_group rightGroup;

extern digital_out Scraper;
extern digital_out midGoal;
extern digital_out defenseRight;
extern digital_out defenseLeft;

extern motor IntakeSystem;
extern motor OutTakeSystem;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );