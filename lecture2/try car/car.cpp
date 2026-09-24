#include "car.hpp"

#include <iostream>

Car::Car()
{
    count_ = 0;
    std::cout << "Car 构造完成" << std::endl;
}

Car::~Car()
{
    std::cout << "car destructor! It has been read " << count_ << " times" << std::endl;
}

void Car::run()
{
    count_++;
    std::cout << "这是一个 car 类!" << std::endl;
}