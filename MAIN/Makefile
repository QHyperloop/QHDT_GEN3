CC = gcc
CFLAGS = -Wall -pthread -Wextra -g
LFLAGS = -lpigpio -lrt -lm -ljson-c -lwebsockets -lpthread

TARGET = hyperloop
OBJS = main.o control.o

default: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS) $(LFLAGS)

main.o: main.c control.h
	$(CC) $(CFLAGS) -c main.c -o main.o

control.o: control.c control.h
	$(CC) $(CFLAGS) -c control.c -o control.o

# Clean up
clean:
	$(RM) $(TARGET) *.o *~
