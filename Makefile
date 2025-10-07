all: example

example: example.cpp ymfm_ym2612.hpp
	g++ -o example example.cpp
