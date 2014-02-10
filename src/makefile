IDIR =../include

CC =g++-4.8 -std=c++11 -Wall -O3 -pthread
CFLAGS =-I$(IDIR)

ODIR=obj
LDIR =../lib

LIBS=

_DEPS = Pathfinder.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = Pathfinder.o PathfinderUnitTest.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))


$(ODIR)/%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

PathfinderUnitTest: $(OBJ) ;
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 