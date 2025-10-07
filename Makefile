all: example
	./example test.vgm test.wav
	hexdump -C test.wav

example: example.cpp ymfm_opn.hpp
	g++ -o example example.cpp
