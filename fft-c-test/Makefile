CFLAGS = -Wshadow -O3 -g -march=native
LDLIBS = -lm

all:
	gcc example.c -o example fft.c $(LDLIBS) $(CFLAGS)
	./example

# example: example.o fft.o

clean:
	$(RM) *.o
	$(RM) example fft *.exe