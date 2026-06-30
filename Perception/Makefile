CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -I.

OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4)
OPENCV_LIBS   := $(shell pkg-config --libs opencv4)

TARGET = bin/perception
BUILD_DIR = build

SRCS = Main/main.cpp \
	   Cone/cone.cpp \
	   Keypoints/keypoints.cpp\
	   Depth/depth.cpp
       #Calibration/calibration.cpp \
       #Rectification/rectification.cpp \
       

OBJS = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(SRCS))

all: perception

perception: $(OBJS)
	@mkdir -p $(dir $(TARGET))
	$(CXX) $(OBJS) -o $(TARGET) $(OPENCV_LIBS)

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -c $< -o $@

debug: CXXFLAGS += -g -O0
debug: clean perception

release: CXXFLAGS += -O3 -DNDEBUG
release: clean perception

clean:
	rm -rf $(BUILD_DIR) $(TARGET)

help:
	@echo "Targets:"
	@echo "  make           -> build executable"
	@echo "  make debug     -> debug build"
	@echo "  make release   -> optimized build"
	@echo "  make clean     -> remove artifacts"

.PHONY: all perception debug release clean help