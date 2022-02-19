// inheritance.cpp
#include <iostream>
using namespace std;
class A {
public:
	virtual void alpha() {
		cout << "A:alpha" << endl;
	}
	void beta() {
		cout << "A:beta" << endl;
	}
};
class B : public A {
public:
	virtual void alpha() {
		cout << "B:alpha" << endl;
	}
	void beta() {
		cout << "B:beta" << endl;
	}
};
int main( int argc, char* argv[] ) {
	A *class1, *class2;
	class1 = new A;
	class1->alpha();
	class1->beta();
	class2 = new B;
	class2->alpha();
	// see what happens if you don't declare superclass method virtual
	class2->beta();
	return 0;
}
