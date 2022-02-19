// compilation:
// someFunction1.cpp
//  g++ someFunction.cpp -o someFunction

#include <iostream>
void someFunction(int n) {
	int result = 0;
	if (n <= 0) {
		result = 0; 
	} else {
		for (int i = 1; i <= n; i++) {
			result += i;
		}
	}
	std::cout << result << std::endl;
}
int main( int argc, char* argv[] ) {
	someFunction (10);
	return 0;
}
