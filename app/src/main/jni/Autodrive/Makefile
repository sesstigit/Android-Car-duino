# Makefile to make autodrive Sample C++ application only.
CC=gcc
CXX=g++
RM=rm -f
#CPPFLAGS=-g $(shell root-config --cflags)
CPPFLAGS=-g -std=c++11
#LDFLAGS=-g $(shell root-config --ldflags)
LDFLAGS=-g -std=c++11 
#-v #add this to LDFLAGS for verbosity
#LDLIBS=$(shell root-config --libs)
LDLIBS=-lm

SRCDIR=./src
SRCS=$(SRCDIR)/test_main.cpp $(SRCDIR)/CarSensor.cpp $(SRCDIR)/Car.cpp $(SRCDIR)/ParkingManeuver.cpp
OBJS=$(subst .cpp,.o,$(SRCS))

all: test_main docs

test_main: $(OBJS)
	$(CXX) $(LDFLAGS) -o test_main $(OBJS) $(LDLIBS) 

test_main.o: test_main.cpp Autodrive.h

CarSensor.o: CarSensor.h CarSensor.cpp
	
Car.o: Car.h Car.cpp 

ParkingManeuver.o: ParkingManeuver.h ParkingManeuver.cpp 

doxyfile.inc: Makefile
	echo INPUT = $(SRCDIR) > doxyfile.inc
	echo FILE_PATTERNS = *.h $(SRCS) >> doxyfile.inc

docs: doxyfile.inc $(SRCS)
	@doxygen
	@open doxygen/html/index.html

clean:
	$(RM) $(OBJS)

distclean: clean
	$(RM) test_main
