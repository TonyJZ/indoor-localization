DLL=registration

include ../indoor_locationMake.opt

SRC_DIR= $(indoorLoc_SRC)/$(DLL)

_OBJS=build_IntersectionGraph.o	fine_registration.o ipGraphMatching.o

OBJS= $(patsubst %,$(OBJ_DIR)/%,$(_OBJS))

$(COMMON_DLL): $(OBJS)
	$(LD) -s -shared $(LNKFLAGS) $(OBJS) -lCGAL -lm -o  $(COMMON_DLL)  

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) -c $(CXXFLAGS) -fpermissive -o $@ $< 

default:        $(COMMON_DLL)

clean:
	$(RM) $(OBJ_DIR)/*.o
