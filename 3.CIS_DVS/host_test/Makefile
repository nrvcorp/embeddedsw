# ────────────────────────────────────────────────────────────────
# Makefile: sync → clean → high‑performance build
# ────────────────────────────────────────────────────────────────

CXX            := g++
SRC            := ./src/*.cpp
TARGET         := main_c

# OpenCV 플래그 (컴파일+링크)
OPENCV_FLAGS   := $(shell pkg-config --cflags --libs opencv4)
# X11 링크 플래그 (여러 쓰레드에서 imshow 사용)
X11_FLAGS      := -lX11
# zlib
ZLIB_FLAGS     := -lz

PTHREAD_FLAGS  := -pthread

# ── Performance optimization flags ─────────────────────────────
OPT_FLAGS      := -std=c++17 \
                  -O3 -Ofast -march=native -flto \
                  -funroll-loops -fpeel-loops -fstrict-aliasing \
                  -fopenmp 

DEBUG_OPT_FLAGS      := -std=c++17 \
                  -O2 -Ofast -march=native -flto \
                  -funroll-loops -fpeel-loops -fstrict-aliasing \
                  -fopenmp -g

.PHONY: all clean

all: clean
	@echo "[1/2] syncing filesystem buffers…"
	sync
	@echo "[2/2] building $(TARGET)"
	$(CXX) $(OPT_FLAGS) $(SRC) -o $(TARGET) \
	  $(OPENCV_FLAGS) $(X11_FLAGS) $(ZLIB_FLAGS) $(PTHREAD_FLAGS)

debug: clean
	@echo "[1/2] syncing filesystem buffers…"
	sync
	@echo "[2/2] building (Debug) $(TARGET)"
	$(CXX) $(DEBUG_OPT_FLAGS) $(SRC) -o $(TARGET) \
	  $(OPENCV_FLAGS) $(X11_FLAGS) $(ZLIB_FLAGS) $(PTHREAD_FLAGS)

clean:
	@echo "Cleaning previous build…"
	rm -f $(TARGET)


#CXX = g++
#SRC = ./src/*.cpp
#TARGET = main_c
#OPENCV_FLAGS = `pkg-config --cflags --libs opencv4`
#PTHREAD_FLAGS = -pthread

## 최적화 플래그
#OPT_FLAGS = -O2 -std=c++17 -march=native -fopenmp
##OPT_FLAGS = -O3 -march=native -flto -ffast-math -fopenmp

#all:
#	sync
#	make clean
#	$(CXX)  $(SRC)  -o $(TARGET) $(OPT_FLAGS) $(OPENCV_FLAGS) $(PTHREAD_FLAGS)

#clean:
#	rm -f $(TARGET)
