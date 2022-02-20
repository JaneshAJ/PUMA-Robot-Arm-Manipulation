// call_by_reference.cpp
#include <iostream>
using namespace std;
struct Variables {
	double input;
	double output;
};
void control1(Variables v) {
	v.output = 2*v.input;
}
void control2(Variables& v) {
	v.output = 3*v.input;
}
void control3(Variables& v) {
	Variables vars; 
	vars.output = 5;
	v = vars;  // assignment operator shallow-copies members
	// this is not a reassignment of the pointer! try
	// cout << &v << endl; 
	// cout << &vars << endl;
	// and comment in the last line of main.cpp
}
int main( int argc, char* argv[] ) {
	Variables vars;
	vars.input=1.0; vars.output=0.0;
	control1(vars);
	cout << vars.output << endl;
	control2(vars);
	cout << vars.output << endl;
	//control3(vars);
	//cout << vars.output << endl;
	// cout << &vars << endl;
}