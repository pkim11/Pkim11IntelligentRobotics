CC=g++
IDIR=../include/
IDIRS=-I../include/
LDIR=-L../lib/
CFLAGS=-std=c++14 -Wall -O2 $(IDIRS) $(LDIR) -fopenmp
ODIR=../obj/
SRCDIR=

LIBS=-lIRLibrary
EXEC_PATH=../bin/

DEPS=../lib/libIRLibrary.a

all: $(EXEC_PATH)simple2R $(EXEC_PATH)mod2R  

$(EXEC_PATH)simple2R: $(SRCDIR)simple2R.cpp $(DEPS) | $(EXEC_PATH)
	$(CC) $(CFLAGS) $< $(LIBS) -o $@ 

$(EXEC_PATH)mod2R: $(SRCDIR)mod2R.cpp $(DEPS)
	$(CC) $(CFLAGS) $< $(LIBS) -o $@ 

$(EXEC_PATH):
	mkdir -p $(EXEC_PATH)


.PHONY: clean

clean:
	rm  $(EXEC_PATH)* 
