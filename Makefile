all: example
	./example test.vgm test.wav

clean:
	rm -f example

example: example.cpp ymfm_opn2.hpp
	g++ -std=c++17 -O2 -o example example.cpp
