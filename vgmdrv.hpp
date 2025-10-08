
// VGM Driver for ymfm/OPN2
// BSD 3-Clause License
//
// Copyright (c) 2025, Yoji Suzuki
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "ymfm_opn2.hpp"

// we use an int64_t as emulated time, as a 32.32 fixed point value
using emulated_time = int64_t;

enum class ChipType {
    YM2612,
    Unsupported
};

#define YM2612_QUEUE_CAPACITY 4096

struct ChipEntry {
    int offset;
    const char* name;
    ChipType type;
};

static const ChipEntry CHIP_TABLE[] = {
    {0x2C, "YM2612", ChipType::YM2612},
    {0x30, "YM2151", ChipType::Unsupported},
    {0x38, "Sega_PCM", ChipType::Unsupported},
    {0x40, "RF5C68", ChipType::Unsupported},
    {0x44, "YM2203", ChipType::Unsupported},
    {0x48, "YM2608", ChipType::Unsupported},
    {0x4C, "YM2610/B", ChipType::Unsupported},
    {0x50, "YM3812", ChipType::Unsupported},
    {0x54, "YM3526", ChipType::Unsupported},
    {0x58, "Y8950", ChipType::Unsupported},
    {0x5C, "YMF262", ChipType::Unsupported},
    {0x60, "YMF278B", ChipType::Unsupported},
    {0x64, "YMF271", ChipType::Unsupported},
    {0x68, "YMZ280B", ChipType::Unsupported},
    {0x6C, "RF5C164", ChipType::Unsupported},
    {0x70, "PWM", ChipType::Unsupported},
    {0x74, "AY8910", ChipType::Unsupported},
    {0x80, "GB_DMG", ChipType::Unsupported},
    {0x84, "NES_APU", ChipType::Unsupported},
    {0x88, "MultiPCM", ChipType::Unsupported},
    {0x8C, "uPD7759", ChipType::Unsupported},
    {0x90, "OKIM6258", ChipType::Unsupported},
    {0x98, "OKIM6295", ChipType::Unsupported},
    {0x9C, "uPD7759", ChipType::Unsupported},
    {0xA0, "K054539", ChipType::Unsupported},
    {0xA4, "HuC6280", ChipType::Unsupported},
    {0xA8, "C140", ChipType::Unsupported},
    {0xAC, "K053260", ChipType::Unsupported},
    {0xB0, "Pokey", ChipType::Unsupported},
    {0xB4, "QSound", ChipType::Unsupported},
    {0xB8, "SCSP", ChipType::Unsupported},
    {0xC0, "WonderSwan", ChipType::Unsupported},
    {0xC4, "VSU", ChipType::Unsupported},
    {0xC8, "SAA1099", ChipType::Unsupported},
    {0xCC, "ES5503", ChipType::Unsupported},
    {0xD0, "ES5506", ChipType::Unsupported},
    {0xD8, "X1-010", ChipType::Unsupported},
    {0xDC, "C352", ChipType::Unsupported},
    {0xE0, "GA20", ChipType::Unsupported},
    {0xE4, "Mikey", ChipType::Unsupported},
};

static const ChipEntry* find_chip_entry(int offset)
{
    unsigned int count = (unsigned int)(sizeof(CHIP_TABLE) / sizeof(CHIP_TABLE[0]));
    for (unsigned int index = 0; index < count; index++) {
        if (CHIP_TABLE[index].offset == offset)
            return &CHIP_TABLE[index];
    }
    return nullptr;
}

struct Ym2612QueueEntry {
    uint32_t reg;
    uint8_t data;
};

struct Ym2612Queue {
    Ym2612QueueEntry entries[YM2612_QUEUE_CAPACITY];
    int head;
    int tail;
    int count;
};

static inline void ym2612_queue_init(Ym2612Queue* queue)
{
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}

static inline void ym2612_queue_clear(Ym2612Queue* queue)
{
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}

static inline int ym2612_queue_empty(const Ym2612Queue* queue)
{
    return queue->count == 0;
}

static inline Ym2612QueueEntry ym2612_queue_front(const Ym2612Queue* queue)
{
    Ym2612QueueEntry empty = {0, 0};
    if (queue->count == 0)
        return empty;
    return queue->entries[queue->head];
}

static inline void ym2612_queue_pop(Ym2612Queue* queue)
{
    if (queue->count == 0)
        return;
    queue->head = (queue->head + 1) % YM2612_QUEUE_CAPACITY;
    queue->count--;
}

static inline void ym2612_queue_push(Ym2612Queue* queue, uint32_t reg, uint8_t data)
{
    if (queue->count >= YM2612_QUEUE_CAPACITY)
        return;
    queue->entries[queue->tail].reg = reg;
    queue->entries[queue->tail].data = data;
    queue->tail = (queue->tail + 1) % YM2612_QUEUE_CAPACITY;
    queue->count++;
}

class VgmDriver : public ymfm::ymfm_interface
{
  private:
    ymfm::ym2612 ym2612;
    Ym2612Queue ym2612_queue;

    struct Context {
        uint32_t version;
        const uint8_t* data;
        size_t size;
        int32_t cursor;
        int32_t loopOffset;
        int32_t wait;
        uint32_t loopCount;
        bool end;
        emulated_time output_start;
        emulated_time pos;
        emulated_time step;
    } vgm;

    emulated_time output_step;
    uint32_t ym2612_clock;
    bool ym2612_present;
    int channels;

  public:
    VgmDriver(int samples = 44100, int channels = 2) : ym2612(*this)
    {
        ym2612_queue_init(&this->ym2612_queue);
        this->output_step = 0x100000000ull / samples;
        this->channels = channels;
        this->ym2612_clock = 0;
        this->ym2612_present = false;
        this->reset();
    }

    ~VgmDriver()
    {
    }

    void reset()
    {
        memset(&this->vgm, 0, sizeof(this->vgm));
        this->ym2612_clock = 0;
        this->ym2612_present = false;
        this->ym2612.reset();
        ym2612_queue_clear(&this->ym2612_queue);
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
        for (int offset = 0x2C; offset < 0xE8; offset += 4) {
            const ChipEntry* entry = find_chip_entry(offset);
            if (entry == nullptr)
                continue;
            if ((size_t)(entry->offset + 4) > size)
                continue;

            uint32_t clocks = 0;
            memcpy(&clocks, &data[entry->offset], 4);
            if (clocks != 0) {
                printf("Detected %s: clocks=%uHz ", entry->name, clocks);
                if (entry->type != ChipType::Unsupported) {
                    printf("<supported>\n");
                    detect_supported = true;
                    if (entry->type == ChipType::YM2612) {
                        this->ym2612_clock = clocks;
                        this->ym2612_present = true;
                        vgm.step = 0x100000000ull / this->ym2612.sample_rate(clocks);
                    }
                } else {
                    printf("<unsupported!>\n");
                    detect_unsupported = true;
                }
            }
        }
        if (detect_unsupported || !detect_supported || !this->ym2612_present) {
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
            if (2 <= this->channels && (cursor + 1) < samples) {
                buf[cursor + 1] = 0;
            }
            if (this->ym2612_present) {
                uint32_t addr1 = 0xffff, addr2 = 0xffff;
                uint8_t data1 = 0, data2 = 0;

                if (!ym2612_queue_empty(&this->ym2612_queue)) {
                    Ym2612QueueEntry front = ym2612_queue_front(&this->ym2612_queue);
                    addr1 = 0 + 2 * ((front.reg >> 8) & 3);
                    data1 = (uint8_t)(front.reg & 0xff);
                    addr2 = addr1 + 1;
                    data2 = front.data;
                    ym2612_queue_pop(&this->ym2612_queue);
                }

                if (addr1 != 0xffff) {
                    ym2612.write(addr1, data1);
                    ym2612.write(addr2, data2);
                }

                ymfm::ym2612::output_data out;
                for (; vgm.pos <= vgm.output_start; vgm.pos += vgm.step) {
                    ym2612.generate(&out);
                }
                vgm.output_start += output_step;
                if (this->channels < 2) {
                    buf[cursor] += out.data[0];
                    cursor += 1;
                } else if ((cursor + 1) < samples) {
                    buf[cursor] += out.data[0];
                    buf[cursor + 1] += out.data[1];
                    cursor += 2;
                } else {
                    cursor += 1;
                }
            } else {
                cursor += (this->channels < 2) ? 1 : 2;
            }
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
        while (vgm.wait < 1) {
            uint8_t cmd = vgm.data[vgm.cursor++];
            switch (cmd) {
                case 0x52:
                case 0xA2: {
                    // YM2612 port 0, write value dd to register aa
                    uint32_t reg = vgm.data[vgm.cursor++];
                    uint8_t data = vgm.data[vgm.cursor++];
                    ym2612_queue_push(&this->ym2612_queue, reg, data);
                    break;
                }
                case 0x53:
                case 0xA3: {
                    // YM2612 port 1, write value dd to register aa
                    uint32_t reg = vgm.data[vgm.cursor++];
                    uint8_t data = vgm.data[vgm.cursor++];
                    ym2612_queue_push(&this->ym2612_queue, reg | 0x100, data);
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
