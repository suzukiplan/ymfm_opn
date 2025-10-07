all: example
	./example test.vgm test.wav

example: example.cpp ymfm_opn.hpp
	g++ -O2 -o example example.cpp
