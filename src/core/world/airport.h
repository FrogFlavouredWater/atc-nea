#pragma once

#include <raylib.h>
#include <string>
#include <vector>

struct Airport
{
    std::string name;
    Vector2 position;
    double runwayHeading;
    int runwayLength = 60;
    int localiserLength = 200;

    void render() const;
};