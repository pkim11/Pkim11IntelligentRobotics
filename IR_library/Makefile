CXX=g++
IDIR=include/
IDIRS=-Iinclude/
LDIR=-Llib/ 
CFLAGS=-std=c++11 -Wall -O2 $(IDIRS) $(LDIR) -fopenmp -fPIC
ODIR=obj/
SRCDIR=src/

LIBS=
EXEC_PATH=bin/
LIB_PATH=lib/

_DEPS=Planar2R.hpp Planar2Rmod.hpp Planar3R.hpp Spatial3R.hpp RigidBodyMotion.hpp TypeDefs.hpp ForwardKinematics.hpp
DEPS=$(patsubst %,$(IDIR)%,$(_DEPS))

_OBJ_ALL=Planar2R.o Planar2Rmod.o Planar3R.o Spatial3R.o
OBJ_ALL=$(patsubst %,$(ODIR)%,$(_OBJ_ALL))

all: $(LIB_PATH)libIRLibrary.a  
	mkdir -p $(ODIR)
	mkdir -p $(LIB_PATH)
	cd main && $(MAKE)

$(ODIR):
	mkdir -p $(ODIR)

$(LIB_PATH):
	mkdir -p $(LIB_PATH)

$(ODIR)%.o: $(SRCDIR)%.cpp $(DEPS) | $(ODIR) $(LIB_PATH)
	$(CXX) -c -o $@ $< $(CFLAGS)

$(LIB_PATH)libIRLibrary.a: $(OBJ_ALL)
	ar rcs $@ $^
	
.PHONY: clean

clean:
	rm -f $(ODIR)*.o *~ $(EXEC_PATH)* $(LIB_PATH)*
