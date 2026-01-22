#include <iostream>
#include <nlohmann/json.hpp>
#include <fstream>
#include "utils.h"
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using std::cout, std::string;

int parseJSON(string input){
    std::ifstream f(input);

    if (input.empty()) {
        return -1;
    }

    if (!f.is_open()) {
        cout << "Could not open the file: " << input << '\n';
        return -1;
    }

    json data;
    f >> data;

    // Just validating it reads
    // cout << data.dump(4) << '\n';
    return 0;
}

int centerHeight(int max_y, int obj_height)
{
    return (max_y - obj_height) / 2;
}

int centerWidth(int max_x, int obj_width)
{
    return (max_x - obj_width) / 2;
}