CXX = g++
CXXFLAGS = -std=c++17 -O3 -Wall -Wextra -Icore
BUILD_DIR = build

TARGET = $(BUILD_DIR)/fluid_sim_cli

SRCS = core/fluid_sim.cpp cli/main_cli.cpp

all: $(TARGET)

$(TARGET): $(SRCS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)
	@echo "Збірка успішна: $(TARGET)"

benchmark: $(TARGET)
	./$(TARGET) --benchmark

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all benchmark clean
