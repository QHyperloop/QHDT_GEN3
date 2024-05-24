CC = gcc
CFLAGS = -Wall -o -Wextra -g -c
LFLAGS = -lpigpio -lrt

default: hyperloop

hyperloop:	main.o esc.o relay.o can.o temperature.o
	$(CC) $(CFLAGS) -o hyperloop main.o esc.o relay.o can.o temperature.o

main.o: main.c main.h
	$(CC) $(CFLAGS) -c main.c $(LFLAGS)

esc.o: esc.c esc.h
	$(CC) $(CFLAGS) -c esc.c $(LFLAGS)

relay.o: relay.c relay.h
	$(CC) $(CFLAGS) -c relay.c $(LFLAGS)

can.o: can.c can.h
	$(CC) $(CFLAGS) -c can.c $(LFLAGS)

temperature.o: temperature.c temperature.h
	$(CC) $(CFLAGS) -c temperature.c $(LFLAGS)

clean:
	$(RM) hyperloop *.o *~