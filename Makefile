
CC=gcc
CFLAGS=-O3 -g -Wall `pkg-config --cflags libhackrf`
LDFLAGS=-g -lm `pkg-config --libs libhackrf`

all: ukhasnet_tx

ukhasnet_tx: ukhasnet_tx.o
	$(CC) $(LDFLAGS) ukhasnet_tx.o -o ukhasnet_tx

.c.o:
	$(CC) $(CFLAGS) -c $< -o $@

install: all
	mkdir -p ${DESTDIR}/usr/bin
	install -m 755 ukhasnet_tx ${DESTDIR}/usr/bin

clean:
	rm -f *.o ukhasnet_tx

