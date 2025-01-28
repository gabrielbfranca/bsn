#define BOOST_BIND_GLOBAL_PLACEHOLDERS // Avoid warnings with Python 3
#include <boost/python.hpp>

class MyClass
{
public:
    MyClass(int v = 0) : value(v) {}
    void increment() { ++value; }
    int getValue() const { return value; }

private:
    int value;
};

BOOST_PYTHON_MODULE(my_component)
{
    using namespace boost::python;

    class_<MyClass>("MyClass", init<int>())    // Bind the MyClass constructor
        .def("increment", &MyClass::increment) // Bind the increment method
        .def("getValue", &MyClass::getValue);  // Bind the getValue method
}
