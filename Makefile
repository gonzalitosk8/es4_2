ENGINE ?= ENGINE_3_CYL

CC = clang

CFLAGS = -std=c23 -O3 -ffast-math -march=native -Wall -Wextra -Wpedantic -DENSIM4_MONITOR_REFRESH_RATE_HZ=60 -DSDL_DISABLE_OLD_NAMES
LDFLAGS = -lm -lSDL3

BIN = ensim4
SRC = src/main.c src/cJSON.c

all:
	$(CC) $(CFLAGS) $(SRC) $(LDFLAGS) -D$(ENGINE) -o $(BIN)
	@echo "Compilado con ENGINE=$(ENGINE)"

vroom: all
	./$(BIN)

clean:
	rm -f $(BIN)

.PHONY: all vroom clean