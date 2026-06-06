# Polargraph project Makefile
#
# Pipeline:
#   [image] --(rasterize.py)--> SVG --(painter)--> polylines --(sim)--> frames --> UI
#
# Host tools are C++17 binaries built into the repo root (where the Python
# servers expect them). Firmware is built/uploaded via PlatformIO.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall
PORT     ?= 8001

# Host C++ tools (output lives at repo root; servers look for ./painter, ./sim)
PAINTER     := painter
SIM         := sim
PAINTER_SRC := controller/painter.cpp
SIM_SRC     := controller/sim.cpp
SIM_DEPS    := controller/EncoderState.h controller/PolargraphState.h controller/Geometry.h

.PHONY: all tools clean firmware upload monitor sim-ui ui help

all: tools

tools: $(PAINTER) $(SIM)  ## Build both host tools (painter + sim)

$(PAINTER): $(PAINTER_SRC)
	$(CXX) $(CXXFLAGS) $< -o $@

$(SIM): $(SIM_SRC) $(SIM_DEPS)
	$(CXX) $(CXXFLAGS) $(SIM_SRC) -o $@

sim-ui: tools  ## Run the full simulator UI (rasterize->painter->sim) on $(PORT)
	PORT=$(PORT) python3 simulator/server.py

ui: $(PAINTER)  ## Run the lightweight painter-preview UI (port 8000)
	python3 ui/server.py

firmware:  ## Build the Arduino/AVR firmware via PlatformIO
	pio run -d firmware

upload:  ## Build and upload firmware to the board
	pio run -d firmware -t upload

monitor:  ## Open the PlatformIO serial monitor
	pio device monitor -d firmware

clean:  ## Remove built host tools and PlatformIO build artifacts
	rm -f $(PAINTER) $(SIM)
	rm -rf firmware/.pio

help:  ## Show this help
	@grep -E '^[a-zA-Z0-9_-]+:.*## ' $(MAKEFILE_LIST) \
		| sed -E 's/:.*## /\t/' \
		| awk -F'\t' '{printf "  \033[36m%-10s\033[0m %s\n", $$1, $$2}'
