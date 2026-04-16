//
// Created by cofer on 29/9/25.
//

#include <iostream>

void printRadians(double radians) {
    std::cout << "Radians: " << radians;
}

int main() {
    std::cout << "This is task B" << std::endl;

    int angleInDegrees = 180;
    printRadians([](int degrees) { return (3.14/180)*degrees; }(angleInDegrees));

    printRadians([angleInDegrees]() { return (3.14/180)*angleInDegrees; }());
    return 0;
}