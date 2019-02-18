#include <pybind11/pybind11.h>
#include "../include/TransferValuesToArduino.h"
#include "../include/ArduinoControl.h"

namespace py = pybind11;

PYBIND11_MODULE(lyncs_rover, m)
{
	m.doc() = "pybind11 module";
	m.def("TransferValuesToArduino", &TransferValuesToArduino, "a function which transfers values to your Arduino");
	py::class_<ArduinoControl>(m, "arduino_control")
		.def(py::init<>())
		.def("Init", &ArduinoControl::Init)
		.def("Csearch1", &ArduinoControl::Csearch1)
		.def("Csearch2", &ArduinoControl::Csearch2)
		.def("__repr__", [](const ArduinoControl &p) {
			return "ArduinoControl()";
		});
}
