# Compiler
CXX = g++

# Compile option (C++17 for ROS environment)
CXXFLAGS = -std=c++17 -O2 -Wall

# Include path
INCLUDES = -I../src/ -I/usr/include/eigen3

# Source files and output binary
SRC = test_space_station_simulator.cpp
OUT = test_space_station_simulator

# Build rules
all: $(OUT)

$(OUT): $(SRC)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^

clean:
	rm -f $(OUT)
