all:
	g++ -std=c++11 main.cpp blink_detection.cpp findEyeCorner.cpp eyeTracking.cpp findEyeCenter.cpp helpers.cpp `pkg-config opencv --cflags --libs`-pthread -o executable

clean:
	rm executable
