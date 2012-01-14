CC	:=	gcc
CFLAGS	:=	-Wall -O2 -I../../src/api -I../../src/common

SRCS	:=	../../src/api/scab_api.c ../../src/api/serial.c
OBJS	:=	$(SRCS:.c=.o)

all	: libscab_api.a

libscab_api.a: $(OBJS)
	ar rc $@ $(OBJS)
	ranlib $@

%.o	: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	-rm $(OBJS)

fclean:	clean
	-rm libscab_api.a

.PHONY: all clean fclean
