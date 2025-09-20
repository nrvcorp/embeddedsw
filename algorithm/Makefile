# Compiler and flags
CXX ?= g++
LDFLAGS = -pthread
CXXFLAGS = -std=c++17 -D_FILE_OFFSET_BITS=64 -D_GNU_SOURCE -D_LARGE_FILE_SOURCE $(shell pkg-config --cflags opencv4)
LDLIBS   = -lpng $(shell pkg-config --libs opencv4) -lstdc++fs

# Source and object files
SRC_DIR = src
OBJ_DIR = obj
INCLUDE_DIR = include

SOURCES = $(wildcard $(SRC_DIR)/*.cpp $(SRC_DIR)/*.c)
OBJECTS = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SOURCES)))
DEPS =

# Targets
TARGET = main

# Main target
all: $(TARGET)

# Rule for building executable from object files
$(TARGET): $(OBJECTS)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LDLIBS)

# Rule for building object file from source file
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp $(DEPS)
	@mkdir -p $(dir $@)
	$(CXX) -c -o $@ $< $(CXXFLAGS) -I$(INCLUDE_DIR)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c $(DEPS)
	@mkdir -p $(dir $@)
	$(CXX) -c -o $@ $< $(CXXFLAGS) -I$(INCLUDE_DIR)

# Clean target
clean:
	rm -rf $(TARGET) $(OBJECTS)

# Phony targets
.PHONY: all clean