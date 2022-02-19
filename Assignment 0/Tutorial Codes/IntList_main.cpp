#include <iostream>
#include "IntList.h"

int main( int argc, char* argv[] ) {
	IntList list(10); // declare AND init on STACK
	list.add(5);
	list.add(29);
	std::cout << list.elem(0) << std::endl;

	IntList* list2 = new IntList(10); // declare AND init on STACK
		
	list2->add(29);
	std::cout << list2->elem(0) << std::endl;
	
	return 0;
}