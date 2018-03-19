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
LDLIBS=$(shell pkg-config opencv --libs)

SRCDIR1=./src
SRCDIR2=./src/imageprocessor
SRCS=$(wildcard $(SRCDIR1)/*.cpp) $(wildcard $(SRCDIR2)/*.cpp)
OBJS=$(subst .cpp,.o,$(SRCS))

all: test_main docs

# Perhaps add LDLIBS to this?
test_main: $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LDLIBS)

%.o: %.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

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

CXXFLAGS += -MMD
-include $(OBJ_FILES:.o=.d)
