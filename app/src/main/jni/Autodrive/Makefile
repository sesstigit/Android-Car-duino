# Makefile to make autodrive Sample C++ application only.
CC=gcc
CXX=g++
RM=rm -f
#CPPFLAGS=-g $(shell root-config --cflags)
CPPFLAGS=-g -pthread
#LDFLAGS=-g $(shell root-config --ldflags)
LDFLAGS=-g
#LDLIBS=$(shell root-config --libs)
LDLIBS=-lm

SRCS=src/test_main.cpp src/CarSensor.cpp
OBJS=$(subst .cpp,.o,$(SRCS))

all: test_main docs

docs:
	@doxygen
	@open doxygen/html/index.html

test_main: $(OBJS)
	$(CXX) $(LDFLAGS) -o test_main $(OBJS) $(LDLIBS) 

test_main.o: test_main.cpp CarSensor.h

CarSensor.o: CarSensor.h CarSensor.cpp

clean:
	$(RM) $(OBJS)

distclean: clean
	$(RM) test_main
