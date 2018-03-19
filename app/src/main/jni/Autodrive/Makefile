# Makefile to make autodrive Sample C++ applications only.
# Requires environment variable AUTODRIVE_HOME be set to path of Autodrive root directory.
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

INC=-I$(AUTODRIVE_HOME)/src/
SRCDIR1=$(AUTODRIVE_HOME)/src/
SRCDIR2=$(AUTODRIVE_HOME)/src/imageprocessor/
SRCS=$(wildcard $(SRCDIR1)/*.cpp) $(wildcard $(SRCDIR2)/*.cpp)
OBJS=$(subst .cpp,.o,$(SRCS))

all: Sample/test_main Sample/test_drive docs

# Perhaps add LDLIBS to this?
Sample/test_drive: Sample/test_drive.o $(OBJS)
	$(CXX) $(LDFLAGS) $(INC) -o $@ $^ $(LDLIBS)

Sample/test_main: Sample/test_main.o $(OBJS)
	$(CXX) $(LDFLAGS) $(INC) -o $@ $^ $(LDLIBS)

%.o: %.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(INC) -c -o $@ $<

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
