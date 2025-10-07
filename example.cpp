#include <string>
#include <map>
#include <vector>
#include <iostream>
#include <fstream>
#include "ymfm_opn.hpp"

std::map<int, std::string> chips = {
    {0x2C, "YM2612"},
    {0x30, "YM2151"},
    {0x38, "Sega_PCM"},
    {0x40, "RF5C68"},
    {0x44, "YM2203"},
    {0x48, "YM2608"},
    {0x4C, "YM2610/B"},
    {0x50, "YM3812"},
    {0x54, "YM3526"},
    {0x58, "Y8950"},
    {0x5C, "YMF262"},
    {0x60, "YMF278B"},
    {0x64, "YMF271"},
    {0x68, "YMZ280B"},
    {0x6C, "RF5C164"},
    {0x70, "PWM"},
    {0x74, "AY8910"},
    {0x80, "GB_DMG"},
    {0x84, "NES_APU"},
    {0x88, "MultiPCM"},
    {0x8C, "uPD7759"},
    {0x90, "OKIM6258"},
    {0x98, "OKIM6295"},
    {0x9C, "uPD7759"},
    {0xA0, "K054539"},
    {0xA4, "HuC6280"},
    {0xA8, "C140"},
    {0xAC, "K053260"},
    {0xB0, "Pokey"},
    {0xB4, "QSound"},
    {0xB8, "SCSP"},
    {0xC0, "WonderSwan"},
    {0xC4, "VSU"},
    {0xC8, "SAA1099"},
    {0xCC, "ES5503"},
    {0xD0, "ES5506"},
    {0xD8, "X1-010"},
    {0xDC, "C352"},
    {0xE0, "GA20"},
    {0xE4, "Mikey"},
};

enum class ChipType {
    YM2612,
    Unsupported
};

std::vector<ChipType> chips_supported = {ChipType::YM2612};

static ChipType getChipType(std::string chipName)
{
    if (chipName == "YM2612") {
        return ChipType::YM2612;
    }
    return ChipType::Unsupported;
}

class VgmDriver : public ymfm::ymfm_interface
{
  private:
    ymfm::ym2612 ym2612;

    struct Context {
        uint32_t version;
        const uint8_t* data;
        size_t size;
        int32_t cursor;
        int32_t loopOffset;
        int32_t wait;
        uint32_t loopCount;
        bool end;
    } vgm;

    std::map<ChipType, uint32_t> clocks;

  public:
    VgmDriver() : ym2612(*this)
    {
        this->reset();
    }

    ~VgmDriver()
    {
    }

    void reset()
    {
        memset(&this->vgm, 0, sizeof(this->vgm));
        this->clocks.clear();
        this->ym2612.reset();
    }

    bool load(const uint8_t* data, size_t size)
    {
        this->reset();
        if (size < 0x100) {
            return false;
        }
        if (0 != memcmp("Vgm ", data, 4)) {
            return false;
        }

        memcpy(&vgm.version, &data[0x08], 4);
        if (vgm.version < 0x161) {
            return false;
        }

        vgm.data = data;
        vgm.size = size;

        bool detect_unsupported = false;
        bool detect_supported = false;
        for (int i = 0x2C; i < 0xE8; i += 4) {
            auto it = chips.find(i);
            if (it != chips.end()) {
                uint32_t clocks;
                memcpy(&clocks, &data[it->first], 4);
                if (clocks) {
                    printf("Detected %s clocks: %uHz ", it->second.c_str(), clocks);
                    auto type = getChipType(it->second);
                    if (type != ChipType::Unsupported) {
                        printf("<supported>\n");
                        this->clocks[type] = clocks;
                        switch (type) {
                            case ChipType::YM2612: this->ym2612.sample_rate(clocks); break;
                            case ChipType::Unsupported: break;
                        }
                        detect_supported = true;
                    } else {
                        printf("<unsupported!>\n");
                        detect_unsupported = true;
                    }
                }
            }
        }
        if (detect_unsupported || !detect_supported) {
            return false;
        }

        memcpy(&vgm.cursor, &data[0x34], 4);
        vgm.cursor += 0x34;
        memcpy(&vgm.loopOffset, &data[0x1C], 4);
        vgm.loopOffset += vgm.loopOffset ? 0x1C : 0;
        return true;
    }

    void render(int16_t* buf, int samples)
    {
        if (!vgm.data) {
            memset(buf, 0, samples * 2);
            return;
        }
        int cursor = 0;
        while (cursor < samples) {
            if (vgm.wait < 1) {
                this->execute();
            }
            vgm.wait--;
            buf[cursor] = 0;
            for (auto it = this->clocks.begin(); it != this->clocks.end(); it++) {
                switch (it->first) {
                    case ChipType::YM2612: {
                        ymfm::ym2612::output_data out;
                        ym2612.generate(&out);
                        buf[cursor] += out.data[0]; // output left channel only (mono)
                        break;
                    }
                    case ChipType::Unsupported:
                        break;
                }
            }
            cursor++;
        }
    }

    inline uint32_t getLoopCount() { return this->vgm.loopCount; }
    inline bool isEnded() { return this->vgm.end; }

  private:
    void execute()
    {
        if (!vgm.data || vgm.end) {
            return;
        }
        static uint32_t seq;
        while (vgm.wait < 1) {
            uint8_t cmd = vgm.data[vgm.cursor++];
            switch (cmd) {
                case 0x52:
                case 0xA2: {
                    // YM2612 port 0, write value dd to register aa
                    int aa = vgm.data[vgm.cursor++];
                    int dd = vgm.data[vgm.cursor++];
                    ym2612.write(aa, dd);
                    static uint32_t seq;
                    printf("ym2612.write(0x52): aa=%02X, dd=%02X (%d)\n", aa, dd, ++seq);
                    break;
                }
                case 0x53:
                case 0xA3: {
                    // YM2612 port 1, write value dd to register aa
                    int aa = vgm.data[vgm.cursor++];
                    int dd = vgm.data[vgm.cursor++];
                    ym2612.write(aa | 0x100, dd);
                    static uint32_t seq;
                    printf("ym2612.write(0x53): aa=%02X, dd=%02X (%d)\n", aa, dd, ++seq);
                    break;
                }

                case 0x61: {
                    // Wait nn samples
                    unsigned short nn;
                    memcpy(&nn, &vgm.data[vgm.cursor], 2);
                    vgm.cursor += 2;
                    vgm.wait += nn;
                    break;
                }
                case 0x62: vgm.wait += 735; break;
                case 0x63: vgm.wait += 882; break;
                case 0x66: {
                    // End of sound data
                    if (vgm.loopOffset) {
                        vgm.cursor = vgm.loopOffset;
                        vgm.loopCount++;
                        break;
                    } else {
                        vgm.end = true;
                        return;
                    }
                }

                case 0x90: // Setup Stream Control: 0x90 ss tt pp cc (ignore)
                case 0x91: // Set Stream Data: 0x91 ss dd ll bb (ignore)
                case 0x95: // Start Stream (fast call): 0x95 ss bb bb ff
                    // printf("[DAC]0x%02X: %02X %02X %02X %02X\n", cmd, vgm.data[0], vgm.data[1], vgm.data[2], vgm.data[3]);
                    vgm.cursor += 4;
                    break;

                case 0x92: // Set Stream Frequency: 0x92 ss ff ff ff ff (ignore)
                    // printf("[DAC]0x%02X: %02X %02X %02X %02X %02X\n", cmd, vgm.data[0], vgm.data[1], vgm.data[2], vgm.data[3], vgm.data[4]);
                    vgm.cursor += 5;
                    break;

                case 0x93: // Start Stream: 0x93 ss aa aa aa aa mm ll ll ll ll (ignored)
                    // printf("[DAC]0x%02X: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", cmd, vgm.data[0], vgm.data[1], vgm.data[2], vgm.data[3], vgm.data[4], vgm.data[5], vgm.data[6], vgm.data[7], vgm.data[8], vgm.data[9]);
                    vgm.cursor += 10;
                    break;

                case 0x94: // Stop Stream: 0x94 ss
                    // printf("[DAC]0x%02X: %02X\n", cmd, vgm.data[0]);
                    vgm.cursor++;
                    break;

                default:
                    // unsupported command
                    printf("Detected an unknown VGM command: %02X\n", cmd);
                    vgm.end = true;
                    return;
            }
        }
    }
};

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

    VgmDriver vgmdrv;
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
    wh.ch = 1;
    wh.sample = 44100;
    wh.bps = 88200;
    wh.bsize = 2;
    wh.bits = 16;
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
