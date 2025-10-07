#include <string>
#include <map>
#include <vector>
#include <iostream>
#include <fstream>
#include "ymfm_ym2612.hpp"

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

std::vector<std::string> chips_supported = {"YM2612"};

class VgmDriver : public ymfm::ymfm_interface
{
  private:
    ymfm::ym2612 fm;

    struct Context {
        uint32_t version;
        const uint8_t* data;
        size_t size;
        std::map<std::string, uint32_t> clocks;
        int32_t cursor;
        int32_t loopOffset;
    } vgm;

  public:
    VgmDriver() : fm(*this)
    {
    }

    ~VgmDriver()
    {
    }

    void reset() { this->fm.reset(); }

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
                    bool detect = false;
                    for (auto chip : chips_supported) {
                        if (chip == it->second) {
                            detect = true;
                            break;
                        }
                    }
                    if (detect) {
                        printf("<supported>\n");
                        this->vgm.clocks[it->second] = clocks;
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
        vgm.cursor += 0x40 - 0x0C;
        memcpy(&vgm.loopOffset, &data[0x1C], 4);
        vgm.loopOffset += vgm.loopOffset ? 0x1C : 0;
        return true;
    }

  public:
    //
    // timing and synchronizaton
    //

    // the chip implementation calls this when a write happens to the mode
    // register, which could affect timers and interrupts; our responsibility
    // is to ensure the system is up to date before calling the engine's
    // engine_mode_write() method
    virtual void ymfm_sync_mode_write(uint8_t data) { m_engine->engine_mode_write(data); }

    // the chip implementation calls this when the chip's status has changed,
    // which may affect the interrupt state; our responsibility is to ensure
    // the system is up to date before calling the engine's
    // engine_check_interrupts() method
    virtual void ymfm_sync_check_interrupts() { m_engine->engine_check_interrupts(); }

    // the chip implementation calls this when one of the two internal timers
    // has changed state; our responsibility is to arrange to call the engine's
    // engine_timer_expired() method after the provided number of clocks; if
    // duration_in_clocks is negative, we should cancel any outstanding timers
    virtual void ymfm_set_timer(uint32_t tnum, int32_t duration_in_clocks) {}

    // the chip implementation calls this to indicate that the chip should be
    // considered in a busy state until the given number of clocks has passed;
    // our responsibility is to compute and remember the ending time based on
    // the chip's clock for later checking
    virtual void ymfm_set_busy_end(uint32_t clocks) {}

    // the chip implementation calls this to see if the chip is still currently
    // is a busy state, as specified by a previous call to ymfm_set_busy_end();
    // our responsibility is to compare the current time against the previously
    // noted busy end time and return true if we haven't yet passed it
    virtual bool ymfm_is_busy() { return false; }
};

int main(int argc, char* argv[])
{
    if (argc < 2) {
        puts("usage: example /path/to/file.vgm");
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
    return 0;
}
