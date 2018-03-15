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

SRCDIR=./src
SRCS=$(SRCDIR)/test_main.cpp $(SRCDIR)/CarSensor.cpp $(SRCDIR)/Car.cpp
OBJS=$(subst .cpp,.o,$(SRCS))

all: test_main docs

test_main: $(OBJS)
	$(CXX) $(LDFLAGS) -o test_main $(OBJS) $(LDLIBS) 

test_main.o: test_main.cpp CarSensor.h

CarSensor.o: CarSensor.h CarSensor.cpp

Car.o: Car.h Car.cpp

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
