CFLAGS += -DLIBFTDI -Wno-format
LDFLAGS += -lftdi -lusb

all: blackmagic

blackmagic: $(SRC:.c=.o)
	$(CC) -o $@ $^ $(LDFLAGS)

host_clean:
	-rm -f blackmagic