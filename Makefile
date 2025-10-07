all: example
	./example test.vgm test.wav

example: example.cpp ymfm_ym2612.hpp
	g++ -o example example.cpp
