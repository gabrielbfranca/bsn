#include <boost/python.hpp>
#include "component/g3t1_3/G3T1_3.hpp"

BOOST_PYTHON_MODULE(sensor_module)
{
    using namespace boost::python;
    class_<G3T1_3>("G3T1_3", init<int &, char **, std::string>())
        .def("setUp", &G3T1_3::setUp)
        .def("collect", &G3T1_3::collect)
        .def("process", &G3T1_3::process)
        .def("transfer", &G3T1_3::transfer);
}
