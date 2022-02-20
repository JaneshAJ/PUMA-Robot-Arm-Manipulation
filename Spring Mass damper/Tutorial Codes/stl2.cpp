//stl2.cpp
#include <iostream>
#include <vector>
#include <string>

int main( int argc, char* argv[] ) {
	std::vector<std::vector<int> > int2d;
		// vector of vectors = 2d array!
	// ...add some elements to int2d...
	std::vector<int> row1, row2;
	row1.push_back(3);
	row1.push_back(4);
	row2.push_back(5);
	row2.push_back(6);
	int2d.push_back(row1);
	int2d.push_back(row2);
	std::cout << int2d[0][1] << std::endl;

	std::vector<std::string> slist;
	// ...add some elements to slist...
	slist.push_back("hello");
	slist.push_back("world");
	slist.push_back("!");

	// using iterator to go through list
	std::vector<std::string>::iterator it;	
	for (it=slist.begin(); it!=slist.end(); it++){
	  std::string current = *it;
	  std::cout << current << std::endl;
	}

}