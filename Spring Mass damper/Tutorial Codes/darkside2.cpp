// darkside2.cpp
#include <iostream>
using namespace std;

int main( int argc, char* argv[] ) {
	int size = 5;
	// (int size) initialized from outside
	int* numbers = new int[size];
	numbers[0] = 10; numbers[2] = 5;
 	cout << "A: " << numbers << endl;
 	cout << "B: " << (*numbers) << endl;
	cout << "C: " << numbers[2] << endl;
	cout << "D: " << ((*numbers)+2) << endl;
	cout << "E: " << &(*numbers) << endl;
	cout << "F: " << &size << endl;
	delete[] numbers;
}