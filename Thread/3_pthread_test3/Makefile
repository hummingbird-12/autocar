CC = g++

CFLAGS = -Wall -O2
SRCS = pthread_test3.cpp

PROG = test

OPENCV = `pkg-config --cflags --libs opencv`
LIBS = -lwiringPi -lpthread $(OPENCV)

$(PROG): $(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)

clean:
	rm -rf $(PROG)
