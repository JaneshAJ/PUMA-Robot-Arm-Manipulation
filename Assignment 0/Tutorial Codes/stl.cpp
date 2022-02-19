//stl.cpp
#include <iostream>
#include <vector>
#include <string>

int main( int argc, char* argv[] ) {
	std::string name1("Klaus");
	std::string name2("Peter");
	std::vector<std::string> names;
	
	names.push_back(name1);
	names.push_back(name2);

	std::cout << names[0] << std::endl;
}