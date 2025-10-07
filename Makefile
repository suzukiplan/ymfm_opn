all: example
	./example test.vgm test.wav

example: example.cpp ymfm_opn2.hpp
	g++ -O2 -o example example.cpp
