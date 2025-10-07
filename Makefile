all: example
	./example test.vgm test.wav

example: example.cpp ymfm_opn.hpp
	g++ -o example example.cpp
