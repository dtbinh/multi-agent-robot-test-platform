all:	pattern_tracking contour contour_check contour_standalone

pattern_tracking: pattern_tracking.cpp
	g++  pattern_tracking.cpp -o pattern_tracking `pkg-config opencv --cflags --libs`

contour: contour.cpp
	g++  contour.cpp -o contour `pkg-config opencv --cflags --libs`

contour_check: contour_check.cpp
	g++  contour_check.cpp -o contour_check `pkg-config opencv --cflags --libs` -fno-stack-protector

contour_standalone: contour_standalone.cpp
	g++  contour_standalone.cpp -o contour_standalone `pkg-config opencv --cflags --libs`