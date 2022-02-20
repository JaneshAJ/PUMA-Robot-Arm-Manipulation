// someFunction2.cpp
#include "someFunction2.h"

int someFunction(int n) {
  int result = 0;
  if (n <= 0) {
    result = 0; 
  } else {
    for (int i=1; i <= n; i++) {
	result += i;
    }
  }
  return result;
}
