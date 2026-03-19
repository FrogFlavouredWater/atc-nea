#pragma once
#include <string>

struct Vec2 {
    double x;
    double y;
};

//functiony crap
int centerHeight(int max_y, int obj_height);
int centerWidth(int max_x, int obj_width);

//math utils
double normalizeAngle(double angle);
double getShortestAngleDiff(double target, double current);
