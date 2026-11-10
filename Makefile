CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -Werror -Iinclude
SRC = src/dynamics.c src/integrator.c src/controller.c

.PHONY: all test clean

all: main

main: src/main.c $(SRC)
	$(CC) $(CFLAGS) -o main src/main.c $(SRC) -lm

test_integrator: tests/test_integrator.c $(SRC)
	$(CC) $(CFLAGS) -o test_integrator tests/test_integrator.c $(SRC) -lm

test_dynamics: tests/test_dynamics.c src/dynamics.c src/integrator.c
	$(CC) $(CFLAGS) -o test_dynamics tests/test_dynamics.c src/dynamics.c src/integrator.c -lm

test: test_integrator test_dynamics
	./test_integrator
	./test_dynamics

clean:
	rm -f main test_integrator test_dynamics