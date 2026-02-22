CXX=g++
CXXFLAGS += -O3 -MMD -std=c++20

targets : vanilla  postCOM

vanilla :  BaseSim.o  main.cpp
	$(CXX) $(CXXFLAGS) BaseSim.o  main.cpp -DSIM_CLASS=BaseSim -o vanilla 

postCOM : BaseSim.o PostCOMSim.o main.cpp
	$(CXX) $(CXXFLAGS) BaseSim.o PostCOMSim.o  main.cpp -DSIM_CLASS=PostCOMSim -o postCOM

BaseSim.o :  libs/BaseSim.cpp 
	$(CXX) $(CXXFLAGS) -c libs/BaseSim.cpp -o BaseSim.o

PostCOMSim.o : libs/PostCOMSim.cpp 
	$(CXX) $(CXXFLAGS) -c libs/PostCOMSim.cpp -o PostCOMSim.o




clean :
	rm -r *.o  *.d *.dSYM vanilla postCOM

-include $(wildcard *.d)
