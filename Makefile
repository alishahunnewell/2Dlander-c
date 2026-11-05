CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -Werror -Iinclude
SRC = src/dynamics.c src/integrator.c

.PHONY: all test clean

all: main

main: src/main.c $(SRC)
	$(CC) $(CFLAGS) -o main src/main.c $(SRC) -lm

test_integrator: tests/test_integrator.c $(SRC)
	$(CC) $(CFLAGS) -o test_integrator tests/test_integrator.c $(SRC) -lm

test: test_integrator
	./test_integrator

clean:
	rm -f main test_integrator