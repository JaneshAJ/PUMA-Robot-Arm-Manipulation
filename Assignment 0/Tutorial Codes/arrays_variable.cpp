// arrays_variable.cpp
#include <iostream>
#include <stdlib.h>
int main( int argc, char* argv[] ) {
	if (argc < 2) return -1;
	int size = atoi(argv[1]);
	//int numbers[size];
	int* numbers = new int[size];
	numbers[0] = numbers[1] = 1;
	for (int i = 2; i < size; i++) {
	  numbers[i] = numbers[i-1] + numbers[i-2];
	}
	std::cout << numbers[size-1] << std::endl;
	return 0; 
}
