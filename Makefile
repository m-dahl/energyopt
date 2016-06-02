PROJECT=energyopt
SOURCES=ADOL-C_NLP.cpp ADOL-C_sparseNLP.cpp Constraints.cpp Interface.cpp IO.cpp ipopt.cpp Model.cpp PostProcess.cpp RobotData.cpp RobotModel.cpp SolveByIpopt.cpp utility.cpp Variables.cpp ZoneData.cpp
LIBRARY=nope

INCPATHS=/usr/local/include/activemq-cpp-3.9.2 /usr/include/apr-1.0 /usr/include/coin/
LIBPATHS=/usr/local/lib 
LDFLAGS=-lactivemq-cpp -lipopt -ladolc
CFLAGS=-c -std=c++11 -DHAVE_STDDEF_H -Wno-deprecated-declarations
CC=g++

# ------------ MAGIC BEGINS HERE -------------

# Automatic generation of some important lists
OBJECTS=$(SOURCES:.cpp=.o)
INCFLAGS=$(foreach TMP,$(INCPATHS),-I$(TMP))
LIBFLAGS=$(foreach TMP,$(LIBPATHS),-L$(TMP))

# Set up the output file names for the different output types
ifeq "$(LIBRARY)" "shared"
    BINARY=lib$(PROJECT).so
    LDFLAGS += -shared
else ifeq "$(LIBRARY)" "static"
    BINARY=lib$(PROJECT).a
else
    BINARY=$(PROJECT)
endif

all: $(SOURCES) $(BINARY)

$(BINARY): $(OBJECTS)
    # Link the object files, or archive into a static library
    ifeq "$(LIBRARY)" "static"
	ar rcs $(BINARY) $(OBJECTS)
    else
	$(CC) $(LIBFLAGS) $(OBJECTS) $(LDFLAGS) -o $@
    endif

.cpp.o:
	$(CC) $(INCFLAGS) $(CFLAGS) -fPIC $< -o $@

distclean: clean
	rm -f $(BINARY)

clean:
	rm -f $(OBJECTS)
