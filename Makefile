CXX=clang++
CXXFLAGS +=  -g -O0 -MMD

targets : vanilla 

vanilla :  baseline.o  main.cpp
	$(CXX) $(CXXFLAGS) baseline.o  main.cpp -o vanilla

baseline.o : libs/baseline.cpp
	$(CXX) $(CXXFLAGS) -c libs/baseline.cpp -o baseline.o




clean :
	rm *.o  vanilla

-include $(wildcard *.d)
