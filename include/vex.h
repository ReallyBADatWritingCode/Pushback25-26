#pragma once

#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <functional>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "v5.h"
#include "v5_vcs.h"

#include "NH-Template/PID.h"
#include "NH-Template/vector2.h"
#include "NH-Template/util.h"
#include "NH-Template/odom.h"
#include "NH-Template/drive.h"

#include "robot-config.h"
#include "autons.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)