// const_methods.cpp
#include <iostream>
using namespace std;
class A {
	int alpha;
public:
	virtual int getAlpha() const{
		//alpha = 1; // would give a compiler error!
		return alpha;		
	}
	virtual void setAlpha(int a) {
		alpha = a;
	}
};

int main( int argc, char* argv[] ) {
	A *a;
	a = new A;
	a->setAlpha(5);
	cout << a->getAlpha() << endl;
	return 0;
}
