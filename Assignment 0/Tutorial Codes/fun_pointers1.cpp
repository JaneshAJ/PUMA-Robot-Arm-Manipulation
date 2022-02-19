// call_by_reference.cpp
#include <iostream>
using namespace std;

struct MotorVariables {
	double output1;
	double output2;
};

MotorVariables* control(const double input) {
	MotorVariables results; 	// result is allocated on heap
	results.output1 = 2.*input;
	results.output2 = 4.*input;
	return &results; // never ever do this!
	// data of results will be cleared after 
	// exiting this function
}


int main( int argc, char* argv[] ) {
	MotorVariables* vars = NULL; // set pointer to NULL
	vars = control(10.0);
	cout << "vars " << &vars << endl;
	cout << vars->output1 << endl;
}