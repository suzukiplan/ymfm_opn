# Extract an OPN2 from ymfm

This code extracts only the classes necessary for emulating the OPN2 from [ymfm](https://github.com/aaronsgiles/ymfm/tree/17decfae857b92ab55fbb30ade2287ace095a381), and modifies it into a single-header format while including its dependency code.
 
## How to use

Check the [example.cpp](./example.cpp).

### 1. `#include`

In [example.cpp](./example.cpp), include [vgmdrv.hpp](./vgmdrv.hpp) with `#include`,

```c++
#include "vgmdrv.hpp"
```

### 2. Create an instance

After creating an instance of the `VgmDriver` class by specifying the sampling rate (typically 44100) and the number of channels (1: Mono, 2: Stereo),

```c++
const int sampling_rate = 44100;
const int channels = 1;
VgmDriver vgmdrv(sampling_rate, channels);
```

### 3. Load a `.vgm` file

The `VgmDriver::load` method reads the .vgm file,

```c++
if (!vgmdrv.load(buf.data(), buf.size())) {
    puts("Load failed!");
    return -2;
}
```

### 4. Render the samplies

The `VgmDriver::render` method continues to acquire sample data in chunks of 4096 samples until it detects the end of the music data or a loop.

```c++
std::vector<int16_t> wav;
while (0 == vgmdrv.getLoopCount() && !vgmdrv.isEnded()) {
    int16_t work[4096];
    vgmdrv.render(work, sizeof(work) / 2);
    wav.reserve(wav.size() + sizeof(work) / 2);
    wav.insert(wav.end(), std::begin(work), std::end(work));
}
printf("%zu samples generated.\n", wav.size());
```

> This [example.cpp](./example.cpp) outputs the sampling results simply as a .wav file to eliminate OS dependencies.
>
> In the typical C++ games or SDKs, the implementation involves outputting the sampled data obtained in the `VgmDriver::render` method using SDL2 audio callbacks, DirectSound, or similar methods.

## How to test

You can build and run the application by executing the following command.

```bash
git clone https://github.com/suzukiplan/ymfm_opn2
cd ymfm_opn2
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

## How to integrate to your Project

Add [vgmdrv.hpp](./vgmdrv.hpp) and [ymfm_opn2.hpp](./ymfm_opn2.hpp) to your project and include them with `#include “vgmdrv.hpp”`.

Since there is no code (.c or .cpp) that depends on anything other than the header files, it should be easy to integrate!

## Restrictions

- The VGM commands implemented by vgmdrv are minimal. If your created VGM causes an error with `“Detected an unknown VGM command: xx”`, you must add the necessary commands.
- We haven't verified this yet, but the DAC for channel 6 likely isn't currently supported.

## License

[3-Clause BSD](./LICENSE)
