APP=utest_buildipgraph

MODULE=utest

include ../../indoor_locationMake.opt

SRC_DIR= $(indoorLoc_UTEST)/$(APP)

_OBJS:= utest_buildipgraph.o

OBJS:= $(patsubst %,$(OBJ_DIR)/%,$(_OBJS))

$(APP_EXE): $(OBJS)
	$(LD) -s -L$(BIN_DIR)  $(LNKFLAGS) $(OBJS) -lfilesupport -lregistration -lpcl_common -lpcl_io -lpcl_search -lpcl_octree -lpcl_kdtree\
	 -lpcl_sample_consensus -lpcl_registration -lflann -lboost_system -lboost_filesystem -lgmp -lmpfr -lm  -o  $(APP_EXE)   

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) -c $(CXXFLAGS) -o $@ $<

default:  $(APP_EXE)

clean:
	$(RM) $(OBJ_DIR)/*.o

	

