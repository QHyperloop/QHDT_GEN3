CC = gcc
CFLAGS = -Wall -pthread -o -Wextra -g
LFLAGS = -lpigpio -lrt

default: hyperloop

hyperloop:	main.c main.h
	$(CC) $(CFLAGS) hyperloop main.c $(LFLAGS)

clean:
	$(RM) hyperloop *.o *~