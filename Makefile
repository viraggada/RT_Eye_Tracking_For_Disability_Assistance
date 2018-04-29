all:
	g++ main.cpp findEyeCorner.cpp eyeTracking.cpp findEyeCenter.cpp helpers.cpp `pkg-config opencv --cflags --libs`-pthread -o executable

clean:
	rm executable
