all:	pattern_tracking contour

pattern_tracking: pattern_tracking.cpp
	g++  pattern_tracking.cpp -o pattern_tracking `pkg-config opencv cvblob --cflags --libs`

contour: contour.cpp
	g++  contour.cpp -o contour `pkg-config opencv cvblob --cflags --libs`
