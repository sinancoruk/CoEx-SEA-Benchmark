CC = gcc
CFLAGS  =
LFLAGS = SEALibgen.c SEALibexc.c -lm -l bcm2835
PROGRAMNAME = run
TARGET = SEACont

all: $(TARGET)

$(TARGET): $(TARGET).c
	$(CC) $(CFLAGS) -o $(PROGRAMNAME) $(TARGET).c $(LFLAGS)
