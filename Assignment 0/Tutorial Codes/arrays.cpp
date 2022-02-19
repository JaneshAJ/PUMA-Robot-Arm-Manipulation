// arrays.cpp
#include <iostream>
using namespace std;

int main( int argc, char* argv[] ) {
	int numbers[10];
	numbers[0] = numbers[1] = 1;
	for (int i = 2; i < 10; i++) {
	  numbers[i] = numbers[i-1] + numbers[i-2];
	}
	
	cout << numbers[2] << ", "
	 << numbers[3] << ", " << numbers[4] << endl;

	return 0;
}
