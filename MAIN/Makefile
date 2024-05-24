CC = gcc
CFLAGS = -Wall -o -Wextra -g -c
LFLAGS = -lpigpio -lrt

default: hyperloop

hyperloop:	main.c main.h
	$(CC) $(CFLAGS) -c hyperloop main.c $(LFLAGS)

clean:
	$(RM) hyperloop *.o *~