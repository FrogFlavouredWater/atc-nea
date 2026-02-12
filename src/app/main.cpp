#include "app/engine.h"

int main()
{
    Engine& engine = Engine::getInstance();
    engine.init();
    engine.run();

    return 0;
}