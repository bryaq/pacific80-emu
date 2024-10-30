NAME=pac80emu
OBJS=pac80emu.o i8080.o
VPATH=8080
CPPFLAGS=-D_GNU_SOURCE
CFLAGS=-O3 -std=c99 -Wall -pedantic
LDLIBS=-lSDL2

.PHONY: all clean

all: $(NAME)

$(NAME): $(OBJS)

clean:
	rm -f $(NAME) $(OBJS)
