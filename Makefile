CXX=g++
CXXFLAGS += -O3 -MMD -std=c++20

targets : vanilla  postCOM postCOM_contig iterative dfsOrder bodyBlocking bodyBlocking2

vanilla :  BaseSim.o  main.cpp
	$(CXX) $(CXXFLAGS) BaseSim.o  main.cpp -DSIM_CLASS=BaseSim -o vanilla 

postCOM : BaseSim.o PostCOMSim.o main.cpp
	$(CXX) $(CXXFLAGS) BaseSim.o PostCOMSim.o  main.cpp -DSIM_CLASS=PostCOMSim -o postCOM

postCOM_contig : BaseSim_contig.o PostCOMSim_contig.o main.cpp 
	$(CXX) $(CXXFLAGS) BaseSim_contig.o PostCOMSim_contig.o  main.cpp -DSIM_CLASS=PostCOMSim -DUSE_CONTIGUOUS -o postCOM_contig

iterative : BaseSim_contig.o  IterativeSim.o main.cpp
	$(CXX) $(CXXFLAGS) BaseSim_contig.o IterativeSim.o main.cpp -DSIM_CLASS=IterativeSim -DUSE_CONTIGUOUS -o iterative

dfsOrder : BaseSim_contig.o IterativeSim.o DFSOrderSim.o main.cpp 
	$(CXX) $(CXXFLAGS) BaseSim_contig.o IterativeSim.o DFSOrderSim.o main.cpp -DSIM_CLASS=DFSOrderSim -DUSE_CONTIGUOUS -o dfsOrder

bodyBlocking : BaseSim_contig.o IterativeSim.o DFSOrderSim.o  BodyBlockingSim.o main.cpp
	$(CXX) $(CXXFLAGS) BaseSim_contig.o IterativeSim.o DFSOrderSim.o BodyBlockingSim.o  main.cpp -DSIM_CLASS=BodyBlockingSim -DUSE_CONTIGUOUS -o bodyBlocking

bodyBlocking2 : BaseSim_contig.o IterativeSim.o DFSOrderSim.o  BodyBlockingSim.o main.cpp
	$(CXX) $(CXXFLAGS) BaseSim_contig.o IterativeSim.o DFSOrderSim.o BodyBlockingSim.o  main.cpp -DSIM_CLASS=BodyBlockingSim -DUSE_CONTIGUOUS -DBLOCK_SIZE=16 -o bodyBlocking2

BaseSim.o :  libs/BaseSim.cpp 
	$(CXX) $(CXXFLAGS) -c libs/BaseSim.cpp -o BaseSim.o

PostCOMSim.o : libs/PostCOMSim.cpp 
	$(CXX) $(CXXFLAGS) -c libs/PostCOMSim.cpp -o PostCOMSim.o


BaseSim_contig.o : libs/BaseSim.cpp
	$(CXX) $(CXXFLAGS) -c libs/BaseSim.cpp -DUSE_CONTIGUOUS -o BaseSim_contig.o

PostCOMSim_contig.o : libs/PostCOMSim.cpp 
	$(CXX) $(CXXFLAGS) -c libs/PostCOMSim.cpp -DUSE_CONTIGUOUS -o PostCOMSim_contig.o

IterativeSim.o : libs/IterativeSim.cpp 
	$(CXX) $(CXXFLAGS) -c libs/IterativeSim.cpp -DUSE_CONTIGUOUS -o IterativeSim.o

DFSOrderSim.o : libs/DFSOrderSim.cpp	
	$(CXX) $(CXXFLAGS) -c libs/DFSOrderSim.cpp -DUSE_CONTIGUOUS -o DFSOrderSim.o

BodyBlockingSim.o : libs/BodyBlockingSim.cpp
	$(CXX) $(CXXFLAGS) -c libs/BodyBlockingSim.cpp -DUSE_CONTIGUOUS -o BodyBlockingSim.o







clean :
	rm -r *.o  *.d *.dSYM vanilla postCOM postCOM_contig Iterative bodyBlocking bodyBlocking2

-include $(wildcard *.d)
