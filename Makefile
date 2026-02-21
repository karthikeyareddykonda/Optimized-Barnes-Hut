CXX=g++
CXXFLAGS += -O3 -MMD -std=c++20

targets : vanilla 

vanilla :  baseline.o  main.cpp
	$(CXX) $(CXXFLAGS) baseline.o  main.cpp -o vanilla

baseline.o : libs/baseline.cpp
	$(CXX) $(CXXFLAGS) -c libs/baseline.cpp -o baseline.o




clean :
	rm -r *.o  *.d *.dSYM vanilla

-include $(wildcard *.d)
