#include "car.hpp"
#include <iostream>

int main()
{
    Car benz;
    benz.run();
    benz.run();
    {
        Car bmw;
        bmw.run();
    }
    return 0;
}
