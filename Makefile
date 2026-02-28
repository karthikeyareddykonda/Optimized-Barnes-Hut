CXX=g++
CXXFLAGS += -O3 -MMD -std=c++20 -march=native

targets : vanilla  postCOM contig iterative dfsOrder bodyBlocking bodyBlocking2 avx multi_thread_force multi_thread lock_free

vanilla :  BaseSim.o  main.cpp
	$(CXX) $(CXXFLAGS) BaseSim.o  main.cpp -DSIM_CLASS=BaseSim -o vanilla 

postCOM : BaseSim.o PostCOMSim.o main.cpp
	$(CXX) $(CXXFLAGS) BaseSim.o PostCOMSim.o  main.cpp -DSIM_CLASS=PostCOMSim -o postCOM

contig : BaseSim_contig.o  main.cpp 
	$(CXX) $(CXXFLAGS) BaseSim_contig.o  main.cpp -DSIM_CLASS=BaseSim -DUSE_CONTIGUOUS -o contig

postCOM_contig : BaseSim_contig.o postCOM_contig.o main.cpp 
	$(CXX) $(CXXFLAGS) BaseSim_contig.o postCOM_contig.o  main.cpp -DSIM_CLASS=postCOM -DUSE_CONTIGUOUS -o contig


iterative : BaseSim_contig.o  IterativeSim.o main.cpp
	$(CXX) $(CXXFLAGS) BaseSim_contig.o IterativeSim.o main.cpp -DSIM_CLASS=IterativeSim -DUSE_CONTIGUOUS -o iterative

dfsOrder : BaseSim_contig.o IterativeSim.o DFSOrderSim.o main.cpp 
	$(CXX) $(CXXFLAGS) BaseSim_contig.o IterativeSim.o DFSOrderSim.o main.cpp -DSIM_CLASS=DFSOrderSim -DUSE_CONTIGUOUS -o dfsOrder

bodyBlocking : BaseSim_contig.o IterativeSim.o DFSOrderSim.o  BodyBlockingSim.o main.cpp
	$(CXX) $(CXXFLAGS) BaseSim_contig.o IterativeSim.o DFSOrderSim.o BodyBlockingSim.o  main.cpp -DSIM_CLASS=BodyBlockingSim -DUSE_CONTIGUOUS -o bodyBlocking

bodyBlocking2 : BaseSim_contig.o IterativeSim.o DFSOrderSim.o  BodyBlockingSim.o main.cpp
	$(CXX) $(CXXFLAGS) BaseSim_contig.o IterativeSim.o DFSOrderSim.o BodyBlockingSim.o  main.cpp -DSIM_CLASS=BodyBlockingSim -DUSE_CONTIGUOUS -DBLOCK_SIZE=16 -o bodyBlocking2

multi_thread : BaseSim_mutex.o IterativeSim_mutex.o  MultiThreadedSim.o main.cpp
	$(CXX) $(CXXFLAGS) BaseSim_mutex.o IterativeSim_mutex.o MultiThreadedSim.o  main.cpp -DSIM_CLASS=MultiThreadedSim -DUSE_MUTEX  -o multi_thread

multi_thread_force : BaseSim_contig.o IterativeSim.o DFSOrderSim.o  BodyBlockingSim.o MultiThreadedForceSim.o main.cpp
	$(CXX) $(CXXFLAGS) BaseSim_contig.o IterativeSim.o DFSOrderSim.o BodyBlockingSim.o  MultiThreadedForceSim.o main.cpp -DSIM_CLASS=MultiThreadedForceSim -DUSE_CONTIGUOUS -DBLOCK_SIZE=4 -o multi_thread_force

lock_free : BaseSim_atomic.o IterativeSim_atomic.o LockFreeSim.o main.cpp 
	$(CXX) $(CXXFLAGS) BaseSim_atomic.o IterativeSim_atomic.o LockFreeSim.o main.cpp -DSIM_CLASS=LockFreeSim -DUSE_ATOMIC -o lock_free


avx : BaseSim_contig.o IterativeSim.o DFSOrderSim.o  BodyBlockingSim.o AVXSim.o main.cpp
	$(CXX) $(CXXFLAGS) BaseSim_contig.o IterativeSim.o DFSOrderSim.o BodyBlockingSim.o AVXSim.o main.cpp -DSIM_CLASS=AVXSim -DUSE_CONTIGUOUS -DBLOCK_SIZE=4 -o avx


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

AVXSim.o : libs/AVXSim.cpp
	$(CXX) $(CXXFLAGS) -c libs/AVXSim.cpp -DUSE_CONTIGUOUS -o AVXSim.o

MultiThreadedSim.o : libs/MultiThreadedSim.cpp
	$(CXX) $(CXXFLAGS) -c libs/MultiThreadedSim.cpp -DUSE_MUTEX -o MultiThreadedSim.o

MultiThreadedForceSim.o : libs/MultiThreadedForceSim.cpp 
	$(CXX) $(CXXFLAGS) -c libs/MultiThreadedForceSim.cpp -DUSE_CONTIGUOUS -o MultiThreadedForceSim.o

LockFreeSim.o : libs/LockFreeSim.cpp
	$(CXX) $(CXXFLAGS) -c libs/LockFreeSim.cpp -DUSE_ATOMIC -o LockFreeSim.o

BaseSim_mutex.o : libs/BaseSim.cpp
	$(CXX) $(CXXFLAGS) -c libs/BaseSim.cpp -DUSE_MUTEX -o BaseSim_mutex.o

IterativeSim_mutex.o : libs/IterativeSim.cpp 
	$(CXX) $(CXXFLAGS) -c libs/IterativeSim.cpp -DUSE_MUTEX -o IterativeSim_mutex.o

DFSOrderSim_mutex.o : libs/DFSOrderSim.cpp	
	$(CXX) $(CXXFLAGS) -c libs/DFSOrderSim.cpp -DUSE_MUTEX -o DFSOrderSim_mutex.o

BodyBlockingSim_mutex.o : libs/BodyBlockingSim.cpp
	$(CXX) $(CXXFLAGS) -c libs/BodyBlockingSim.cpp -DUSE_MUTEX -o BodyBlockingSim_mutex.o

BaseSim_atomic.o : libs/BaseSim.cpp
	$(CXX) $(CXXFLAGS) -c libs/BaseSim.cpp -DUSE_ATOMIC -o BaseSim_atomic.o

IterativeSim_atomic.o : libs/IterativeSim.cpp 
	$(CXX) $(CXXFLAGS) -c libs/IterativeSim.cpp -DUSE_ATOMIC -o IterativeSim_atomic.o











clean :
	rm -r *.o  *.d *.dSYM vanilla postCOM contig Iterative bodyBlocking bodyBlocking2 avx multi_thread_force multi_thread lock_free

-include $(wildcard *.d)
