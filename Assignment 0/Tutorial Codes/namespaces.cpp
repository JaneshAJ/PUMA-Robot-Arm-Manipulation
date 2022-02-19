// namespaces.cpp
#include <iostream>
namespace myns {	
	int pow(int n) {
		return n*n;
	}
}

int main( int argc, char* argv[] ) {
	std::cout << myns::pow(5) << std::endl;
	using namespace myns;
	using namespace std;
	cout << pow(5) << endl;

	return 0;
}
