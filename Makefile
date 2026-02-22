CXX=g++
CXXFLAGS += -O3 -MMD -std=c++20

targets : vanilla  postCOM postCOM_contig

vanilla :  BaseSim.o  main.cpp
	$(CXX) $(CXXFLAGS) BaseSim.o  main.cpp -DSIM_CLASS=BaseSim -o vanilla 

postCOM : BaseSim.o PostCOMSim.o main.cpp
	$(CXX) $(CXXFLAGS) BaseSim.o PostCOMSim.o  main.cpp -DSIM_CLASS=PostCOMSim -o postCOM

BaseSim.o :  libs/BaseSim.cpp 
	$(CXX) $(CXXFLAGS) -c libs/BaseSim.cpp -o BaseSim.o

PostCOMSim.o : libs/PostCOMSim.cpp 
	$(CXX) $(CXXFLAGS) -c libs/PostCOMSim.cpp -o PostCOMSim.o

postCOM_contig : BaseSim_contig.o PostCOMSim_contig.o main.cpp 
	$(CXX) $(CXXFLAGS) BaseSim_contig.o PostCOMSim_contig.o  main.cpp -DSIM_CLASS=PostCOMSim -DUSE_CONTIGUOUS -o postCOM_contig

BaseSim_contig.o : libs/BaseSim.cpp
	$(CXX) $(CXXFLAGS) -c libs/BaseSim.cpp -DUSE_CONTIGUOUS -o BaseSim_contig.o

PostCOMSim_contig.o : libs/PostCOMSim.cpp 
	$(CXX) $(CXXFLAGS) -c libs/PostCOMSim.cpp -DUSE_CONTIGUOUS -o PostCOMSim_contig.o







clean :
	rm -r *.o  *.d *.dSYM vanilla postCOM postCOM_contig

-include $(wildcard *.d)
