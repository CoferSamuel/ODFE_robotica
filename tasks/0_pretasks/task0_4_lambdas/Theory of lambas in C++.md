- C++
	- Lambdas
		- Syntax
			- `auto mylambda = [](){}()`
				- `auto mylambda =`
					- Explanation
						- In case you want to save the lambda as an function object and use it later
					- Example
						```c++
						auto add = [](int a, int b) { return a + b; };  
						  
						// Calls the lambda 'add' with arguments 2 and 3, prints the result (5)  
						std::cout << add(2, 3) << std::endl; // prints 5
						```
				- `[]`
					- Explanation
						- To take external variables
					- Example
						```c++
						printRadians([angleInDegrees]() { return (3.14/180)*angleInDegrees; }());
						```
				- `()`
					- To write the input parameters
				- `{}`
					- Code
				- `()`
					- To write the arguments and execute the function for receiving its result
						- Example
							```c++
								int angleInDegrees = 180;  
								printRadians([](int degrees) { return (3.14/180)*degrees; }(angleInDegrees));
							```
					- It is not always needed
						- Explanation
							- It is not always necessary to execute the function
						- Example
							```c++
							std::sort(people.begin(), people.end(), [](const auto& a, const auto& b) {  
							    return std::get<1>(a) < std::get<1>(b);  
							});
							```
							- Explanation
								- In this case the sort function is expecting a function object as an argument, not its result
- END