cmake_minimum_required(VERSION 3.0)

# For OpenCV
set(USING_OPENCV
		Csearch.cpp
)

add_library(open_cv_ ${USING_OPENCV})

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    target_include_directories(open_cv_ PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(open_cv_ ${OpenCV_LIBS})
endif()

#For the shared library
add_subdirectory(pybind11)
add_library(${PROJECT_NAME}   MODULE
	rover_lib.cpp
	CalculateCsearch.cpp
	ArduinoControl.cpp
	TransferValuesToArduino.cpp
	)
target_link_libraries(${PROJECT_NAME} PRIVATE open_cv_ pybind11::module)
set_target_properties(${PROJECT_NAME}  PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
											SUFFIX "${PYTHON_MODULE_EXTENSION}")