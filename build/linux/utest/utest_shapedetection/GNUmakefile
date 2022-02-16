APP=utest_shapedetection

MODULE=utest

include ../../indoor_locationMake.opt

SRC_DIR= $(indoorLoc_UTEST)/$(APP)

_OBJS:= utest_shapedetection.o

OBJS:= $(patsubst %,$(OBJ_DIR)/%,$(_OBJS))

$(APP_EXE): $(OBJS)
	$(LD) -s -L$(BIN_DIR)  $(LNKFLAGS) $(OBJS) -lfilesupport -lshape_detection -lpcl_common -lpcl_io -lpcl_search -lpcl_segmentation -lpcl_octree\
	 -lboost_system -lboost_filesystem -lCGAL -lm  -o  $(APP_EXE)   

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) -c $(CXXFLAGS) -o $@ $<

default:  $(APP_EXE)

clean:
	$(RM) $(OBJ_DIR)/*.o

	

