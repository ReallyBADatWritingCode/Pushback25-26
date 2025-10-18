#pragma once
#include "NH-Template/drive.h"

class Drive;

extern Drive chassis;

extern void default_constants();

extern void red_left();
extern void red_right();
extern void blue_left();
extern void blue_right();

extern void drive_test();
extern void turn_test();
extern void swing_test();
extern void full_test();
extern void odom_test();
extern void tank_odom_test();
extern void holonomic_odom_test();