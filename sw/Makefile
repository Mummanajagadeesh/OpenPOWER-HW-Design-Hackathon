# sw/Makefile
# Simple cross-compile for Microwatt user-space

CC = powerpc-linux-gnu-gcc
CFLAGS = -O2
TARGET = fir_test

all: $(TARGET)

$(TARGET): fir_test.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f $(TARGET)
