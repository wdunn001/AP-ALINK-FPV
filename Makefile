CC=arm-linux-gnueabihf-gcc
TARGET=ap_alink
SRC=ap_alink.c

# Production build (optimized, no debug output)
all:
	$(CC) -O2 -Wall $(SRC) -o $(TARGET) -lpthread -lrt $(LDFLAGS)

# Debug build (with debug output)
debug:
	$(CC) -O2 -Wall -DDEBUG $(SRC) -o $(TARGET)_debug -lpthread -lrt $(LDFLAGS)

clean:
	rm -f $(TARGET) $(TARGET)_debug
