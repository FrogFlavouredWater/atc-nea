#pragma once
#include <string>

//functiony crap
int parseJSON(std::string input);
int centerHeight(int max_y, int obj_height);
int centerWidth(int max_x, int obj_width);

//math utils
float normalizeAngle(float angle);
float getShortestAngleDiff(float target, float current);
