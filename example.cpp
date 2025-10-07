#include "vgmdrv.hpp"

typedef struct {
    char riff[4];
    unsigned int fsize;
    char wave[4];
    char fmt[4];
    unsigned int bnum;
    unsigned short fid;
    unsigned short ch;
    unsigned int sample;
    unsigned int bps;
    unsigned short bsize;
    unsigned short bits;
    char data[4];
    unsigned int dsize;
} WavHeader;

int main(int argc, char* argv[])
{
    if (argc < 3) {
        puts("usage: example /path/to/file.vgm /path/to/output.wav");
        return 1;
    }
    std::ifstream in(argv[1], std::ios::binary | std::ios::ate);
    if (!in) {
        puts("File open error!");
    }
    std::streamsize size = in.tellg();
    in.seekg(0, std::ios::beg);
    std::vector<uint8_t> buf(size);
    if (!in.read(reinterpret_cast<char*>(buf.data()), size)) {
        puts("File not found!");
        return -1;
    }

    const int sampling_rate = 44100;
    const int channels = 1;
    VgmDriver vgmdrv(sampling_rate, channels);
    if (!vgmdrv.load(buf.data(), buf.size())) {
        puts("Load failed!");
        return -2;
    }

    std::vector<int16_t> wav;
    while (0 == vgmdrv.getLoopCount() && !vgmdrv.isEnded()) {
        int16_t work[4096];
        vgmdrv.render(work, sizeof(work) / 2);
        wav.reserve(wav.size() + sizeof(work) / 2);
        wav.insert(wav.end(), std::begin(work), std::end(work));
    }
    printf("%zu samples generated.\n", wav.size());

    puts("Writing a wav file...");
    WavHeader wh;
    memset(&wh, 0, sizeof(wh));
    strncpy(wh.riff, "RIFF", 4);
    strncpy(wh.wave, "WAVE", 4);
    strncpy(wh.fmt, "fmt ", 4);
    strncpy(wh.data, "data", 4);
    wh.bnum = 16;
    wh.fid = 1;
    wh.ch = channels;
    wh.sample = sampling_rate;
    wh.bsize = 2;
    wh.bits = 16;
    wh.bps = wh.sample * wh.ch * (wh.bits / 8);
    wh.dsize = wav.size() * 2;
    wh.fsize = wh.dsize + sizeof(wh) - 8;

    FILE* fw = fopen(argv[2], "wb");
    if (!fw) {
        puts("File open error!");
        return -1;
    }
    fwrite(&wh, 1, sizeof(wh), fw);
    fwrite(wav.data(), 1, wav.size() * 2, fw);
    fclose(fw);

    return 0;
}
