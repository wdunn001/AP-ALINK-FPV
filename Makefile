CC=arm-linux-gnueabihf-gcc
TARGET=ap_alink
SRC=ap_alink.c

all:
	$(CC) $(CFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET)
