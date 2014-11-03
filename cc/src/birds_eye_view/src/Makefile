all:	pattern_tracking contour contour_check cvblob_tracking

pattern_tracking: pattern_tracking.cpp
	g++  pattern_tracking.cpp -o pattern_tracking `pkg-config opencv cvblob --cflags --libs`

contour: contour.cpp
	g++  contour.cpp -o contour `pkg-config opencv cvblob --cflags --libs`

contour_check: contour_check.cpp
	g++  contour_check.cpp -o contour_check `pkg-config opencv cvblob --cflags --libs`

cvblob_tracking: cvblob_tracking.cpp
	g++  cvblob_tracking.cpp -o cvblob_tracking `pkg-config opencv cvblob --cflags --libs`