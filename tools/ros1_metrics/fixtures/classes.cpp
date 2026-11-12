#include "classes.hpp"
int fixtures::Branch::linear(int x) { return x; }
int fixtures::Branch::branch(int x) { if (x>0) return x; return -x; }
