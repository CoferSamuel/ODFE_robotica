//
// Created by cofer on 25/9/25.
//

#include <iostream>

int main() {
    // Lambda function that adds two integers
    // auto: lets the compiler deduce the type of 'add' (it's a function object)
    // []: lambda introducer, no variables captured from outer scope
    // (int a, int b): parameters, two integers
    // { return a + b; }: function body, returns the sum of a and b
    auto add = [](int a, int b) { return a + b; };

    // Calls the lambda 'add' with arguments 2 and 3, prints the result (5)
    std::cout << add(2, 3) << std::endl; // prints 5

    return 0;
}
