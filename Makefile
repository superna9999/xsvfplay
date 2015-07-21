ARCH=$(shell uname -m)

SRCS = memname.c  play.c  scan.c  statename.c  svf.c  tap.c  xsvf.c

# Change dep for linux64
ifeq ($(ARCH),x86_64)
INCDIR=ftd2xx/linux64/
CFLAGS=-pthread -lrt -lm -ldl
else
INCDIR=ftd2xx/linux32/
CFLAGS=-m32 -pthread -lrt -lm -ldl -O3
endif

all:xsvfplay

xsvfplay: $(SRCS) xsvfplay_ftd2xx.c
	$(CC) xsvfplay_ftd2xx.c $(SRCS) -I. -I$(INCDIR) -o xsvfplay -L$(INCDIR) -lftd2xx $(CFLAGS)

clean:
	-rm xsvfplay
