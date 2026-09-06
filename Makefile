CXX := g++
CXXFLAGS := -std=c++11 -Wall -Wextra -I.

LDFLAGS_WII := -lxwiimote

BUILD_DIR := build

WII_SRC := Wiimote/wii4race.cxx
PEDALS_SRC := ArduinoPedals/pedals_forwarder.cxx

WII_BIN := $(BUILD_DIR)/wii4race
PEDALS_BIN := $(BUILD_DIR)/pedals_forwarder

.PHONY: all clean

all: $(WII_BIN) $(PEDALS_BIN)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(WII_BIN): $(WII_SRC) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS_WII)

$(PEDALS_BIN): $(PEDALS_SRC) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -rf $(BUILD_DIR)
