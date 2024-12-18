# Compiler and flags
CXX ?= g++
LDFLAGS = -pthread
CXXFLAGS = -D_FILE_OFFSET_BITS=64 -D_GNU_SOURCE -D_LARGE_FILE_SOURCE $(shell pkg-config --cflags opencv4)
LDLIBS = -lpng $(shell pkg-config --libs opencv4)

# Source files
SOURCES = main.cpp dma_utils.c
OBJECTS = $(SOURCES:.cpp=.o)
DEPS =

# Targets
TARGET = main

# Main target
all: $(TARGET)

# Rule for building executable from object files
$(TARGET): $(OBJECTS)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LDLIBS)

# Rule for building object file from source file
%.o: %.cpp $(DEPS)
	$(CXX) -c -o $@ $< $(CXXFLAGS)

# Clean target
clean:
	rm -rf $(TARGET) $(OBJECTS)

# Phony targets
.PHONY: all clean