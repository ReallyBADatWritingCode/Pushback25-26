#pragma once
#include "NH-Template/drive.h"

class Drive;

extern Drive chassis;

extern void default_constants();

extern void leftAuton();
extern void rightAuton();
extern void drive_test();
extern void turn_test();
extern void swing_test();
extern void full_test();
extern void odom_test();
extern void tank_odom_test();
extern void holonomic_odom_test();