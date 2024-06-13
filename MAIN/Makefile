CC = gcc
CFLAGS = -Wall -pthread -o -Wextra -g
LFLAGS = -lpigpio -lrt -lm -ljson-c -lwebsockets -lpthread

TARGET = hyperloop

default: $(TARGET)

$(TARGET): main.o
	$(CC) $(CFLAGS) -o $(TARGET) main.o $(LFLAGS)

main.o: main.c # main.h
	$(CC) $(CFLAGS) -c main.c -o main.o

# Clean up
clean:
	$(RM) $(TARGET) *.o *~