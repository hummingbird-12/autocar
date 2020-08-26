CC = g++

CFLAGS = -Wall -O2
SRCS = GPIO_test_serial.cpp

PROG = test

OPENCV = `pkg-config --cflags --libs opencv`
LIBS = -lwiringPi $(OPENCV)

$(PROG): $(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)

clean:
	rm -rf $(PROG)
