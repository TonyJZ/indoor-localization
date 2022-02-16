APP=utest_clustering

MODULE=utest

include ../../indoor_locationMake.opt

SRC_DIR= $(indoorLoc_UTEST)/$(APP)

_OBJS:= utest_clustering.o

OBJS:= $(patsubst %,$(OBJ_DIR)/%,$(_OBJS))

$(APP_EXE): $(OBJS)
	$(LD) -s -L$(BIN_DIR)  $(LNKFLAGS) $(OBJS) -lclustering -lfilesupport -lpcl_common -lpcl_io -lpcl_registration\
	 -lboost_system -lm  -o  $(APP_EXE)   

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) -c $(CXXFLAGS) -o $@ $<

default:  $(APP_EXE)

clean:
	$(RM) $(OBJ_DIR)/*.o

	

