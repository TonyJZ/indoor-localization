DLL=clustering

include ../indoor_locationMake.opt

SRC_DIR= $(indoorLoc_SRC)/$(DLL)

_OBJS=cluster_wall_ceiling.o	

OBJS= $(patsubst %,$(OBJ_DIR)/%,$(_OBJS))

$(COMMON_DLL): $(OBJS)
	$(LD) -s -shared $(LNKFLAGS) $(OBJS)  -lpcl_common -lm -o  $(COMMON_DLL)  

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) -c $(CXXFLAGS)  -o $@ $<

default:        $(COMMON_DLL)

clean:
	$(RM) $(OBJ_DIR)/*.o
