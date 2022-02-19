// IntList.cpp
#include "IntList.h"
IntList::IntList(int max_size_) : max_size(max_size_), current_size(0) {
	members = new int[max_size];
}

IntList::~IntList() {
	delete[] members;
}

bool IntList::add(int number) {
	if (this->current_size+1 >= this->max_size) {
		return false;
	}
	members[current_size] = number; // this-> is optional
	current_size++; 
	return true;
}
int IntList::elem(int i) {
	if (i > this->current_size) {
		return -1;
	}
	return members[i];
}