CC=arm-linux-gnueabihf-gcc
TARGET=ap_alink
SRC=ap_alink.c

# Production build (optimized, no debug output)
all:
	$(CC) -O2 -Wall -Wno-unused-result $(SRC) -o $(TARGET) -lpthread -lrt -lm $(LDFLAGS)

# Debug build (with debug output)
debug:
	$(CC) -O2 -Wall -Wno-unused-result -DDEBUG $(SRC) -o $(TARGET)_debug -lpthread -lrt -lm $(LDFLAGS)

clean:
	rm -f $(TARGET) $(TARGET)_debug
