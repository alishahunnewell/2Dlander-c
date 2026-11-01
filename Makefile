CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -Werror -Iinclude

main: src/main.c src/dynamics.c src/integrator.c
	$(CC) $(CFLAGS) -o main src/main.c src/dynamics.c src/integrator.c -lm


clean:
	del /Q main.exe