DLL=filesupport

include ../indoor_locationMake.opt

SRC_DIR= $(indoorLoc_SRC)/$(DLL)

_OBJS=AuxFileIO.o filename_support.o	

OBJS= $(patsubst %,$(OBJ_DIR)/%,$(_OBJS))

$(COMMON_DLL): $(OBJS)
	$(LD) -s -shared $(LNKFLAGS) $(OBJS)  -lm -o  $(COMMON_DLL)  

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) -c $(CXXFLAGS)  -o $@ $<

default:        $(COMMON_DLL)

clean:
	$(RM) $(OBJ_DIR)/*.o
