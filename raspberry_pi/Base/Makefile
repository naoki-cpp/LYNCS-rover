bme280.so:CalculateCsearch.o Csearch.o bme280.cpp 
	clang++ -O3 -Wall -shared -std=c++14 -fPIC `python3 -m pybind11 --includes`CalculateCsearch.o Csearch.o bme280.cpp -o bme280`python3-config --extension-suffix`
Csearch.o: Csearch.cpp Csearch.h
	clang++ -c Csearch.cpp  -lm -lwiringPi -I/usr/local/include/opencv  -I/usr/local/include -L/usr/local/lib -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_ocl -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab -lrt -lpthread -lm -ldl
CalculateCsearch.o: CalculateCsearch.cpp CalculateCsearch.h
	clang++ -c CalculateCsearch.cpp
clean:
	rm Csearch.o CalculateCsearch.o  bme280.so
