#include <iostream>
#include <vector>
#include <tuple>
#include <string>
#include <algorithm> // For std::sort

// Comparison function for sorting by age
bool compareByAge(const std::tuple<std::string, int>& a, const std::tuple<std::string, int>& b) {
    return std::get<1>(a) < std::get<1>(b);
}

int main() {
    // Create a vector of tuples (string, int)
    std::vector<std::tuple<std::string, int>> people;

    // Add some values
    people.emplace_back("Alice", 25);
    people.emplace_back("Bob", 30);
    people.emplace_back("Charlie", 22);
    people.emplace_back("Loro", 21);

    // Iterate through the vector
    for (const auto& person : people) {
        std::string name;
        int age;

        // Unpack the tuple
        std::tie(name, age) = person;

        std::cout << "Name: " << name << ", Age: " << age << std::endl;
    }

    // Accessing directly by index
    auto& first = people[0];
    std::cout << "First person is "
              << std::get<0>(first) << " with age "
              << std::get<1>(first) << std::endl;

    // Exercise: Sort by age
    std::sort(people.begin(), people.end(), compareByAge);
    // NOTE

    std::cout << "\nSorted by age:" << std::endl;
    for (const auto& person : people) {
        std::string name;
        int age;
        std::tie(name, age) = person;
        std::cout << "Name: " << name << ", Age: " << age << std::endl;
    }

    // Sort by age using a lambda
    std::sort(people.begin(), people.end(), [](const auto& a, const auto& b) {
        return std::get<1>(a) < std::get<1>(b);
    });
    std::cout << "\nSorted by age (lambda):" << std::endl;
    for (const auto& person : people) {
        std::string name;
        int age;
        std::tie(name, age) = person;
        std::cout << "Name: " << name << ", Age: " << age << std::endl;
    }

    // Sort by age using a named function
    std::sort(people.begin(), people.end(), compareByAge);
    std::cout << "\nSorted by age (named function):" << std::endl;
    for (const auto& person : people) {
        std::string name;
        int age;
        std::tie(name, age) = person;
        std::cout << "Name: " << name << ", Age: " << age << std::endl;
    }

    return 0;
}
