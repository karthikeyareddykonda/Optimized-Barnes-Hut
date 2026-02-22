CXX=g++
CXXFLAGS += -O3 -MMD -std=c++20

targets : vanilla 

vanilla :  BaseSim.o  main.cpp
	$(CXX) $(CXXFLAGS) BaseSim.o  main.cpp -o vanilla


BaseSim.o :  libs/BaseSim.cpp 
	$(CXX) $(CXXFLAGS) -c libs/BaseSim.cpp -o BaseSim.o





clean :
	rm -r *.o  *.d *.dSYM vanilla

-include $(wildcard *.d)
