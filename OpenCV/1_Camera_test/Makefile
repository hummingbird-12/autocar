CC = g++

CFLAGS = -Wall -O2
SRCS = opencv_camera_test.cpp

PROG = test

OPENCV = `pkg-config --cflags --libs opencv`
LIBS = $(OPENCV)

$(PROG): $(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)

clean:
	rm -rf $(PROG)
