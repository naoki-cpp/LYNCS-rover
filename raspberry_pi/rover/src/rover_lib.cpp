#include <pybind11/pybind11.h>
#include "../include/TransferValuesToArduino.h"

PYBIND11_MODULE(lyncs_rover, m) {
    m.doc() = "pybind11 module"; 
    m.def("TransferValuesToArduino", &TransferValuesToArduino, "a function which transfers values to your Arduino");
}
