// IntList.h
class IntList {
private:
 // private member variables
 int max_size;
 int* members;
 int current_size; //need to store array length 
public:
	IntList(int max_size_);  // constructor
	virtual ~IntList(); // destructor

	// public member functions
	bool add(int number);
	int elem(int i);
};
