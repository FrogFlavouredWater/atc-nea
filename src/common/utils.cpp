#include <iostream>
#include <fstream>
#include <cmath>
#include "common/utils.h"
#include <iostream>
#include <fstream>

using std::cout, std::string;

int centerHeight(int max_y, int obj_height)
{
    return (max_y - obj_height) / 2;
}

int centerWidth(int max_x, int obj_width)
{
    return (max_x - obj_width) / 2;
}

double normalizeAngle(double angle) {
    double result = fmod(angle, 360.0);
    if (result < 0) result += 360.0;
    return result;
}

double getShortestAngleDiff(double target, double current) {
    double diff = target - current;
    diff = fmod(diff + 180.0, 360.0);
    if (diff < 0) diff += 360.0;
    return diff - 180.0;
}

