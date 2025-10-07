# Extract an OPN2 from ymfm

This code extracts only the classes necessary for emulating the OPN2 from [ymfm](https://github.com/aaronsgiles/ymfm/tree/17decfae857b92ab55fbb30ade2287ace095a381), and modifies it into a single-header format while including its dependency code.
 
## How to use

Check the [example.cpp](./example.cpp).

You can build and run the application by executing the following command.

```bash
git clone https://github.com/suzukiplan/ymfm_opn
make
```

When executed, it reads test.vgm from this repository and outputs a .wav file.

```text
g++ -O2 -o example example.cpp
./example test.vgm test.wav
Detected YM2612: clocks=7670454Hz <supported>
3481600 samples generated.
Writing a wav file...
```

## License

[3-Clause BSD](./LICENSE)
