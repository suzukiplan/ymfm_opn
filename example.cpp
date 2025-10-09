#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
    FILE* in = fopen(argv[1], "rb");
    if (!in) {
        puts("File open error!");
        return -1;
    }
    if (fseek(in, 0, SEEK_END) != 0) {
        fclose(in);
        puts("File seek error!");
        return -1;
    }
    long file_size_long = ftell(in);
    if (file_size_long < 0) {
        fclose(in);
        puts("File size error!");
        return -1;
    }
    if (fseek(in, 0, SEEK_SET) != 0) {
        fclose(in);
        puts("File seek error!");
        return -1;
    }
    size_t file_size = (size_t)file_size_long;
    uint8_t* buf = (uint8_t*)malloc(file_size);
    if (!buf) {
        fclose(in);
        puts("Memory allocation error!");
        return -1;
    }
    size_t read_size = fread(buf, 1, file_size, in);
    fclose(in);
    if (read_size != file_size) {
        free(buf);
        puts("File read error!");
        return -1;
    }

    const int sampling_rate = 44100;
    const int channels = 1;
    VgmDriver vgmdrv(sampling_rate, channels);
    if (!vgmdrv.load(buf, file_size)) {
        free(buf);
        puts("Load failed!");
        return -2;
    }

    int16_t* wav = NULL;
    size_t wav_size = 0;
    size_t wav_capacity = 0;
    while (0 == vgmdrv.getLoopCount() && !vgmdrv.isEnded()) {
        int16_t work[4096];
        size_t frame_count = sizeof(work) / sizeof(work[0]);
        vgmdrv.render(work, (int)frame_count);

        size_t required = wav_size + frame_count;
        if (required > wav_capacity) {
            size_t new_capacity = (wav_capacity == 0) ? frame_count : wav_capacity;
            while (new_capacity < required) {
                new_capacity *= 2;
            }
            int16_t* new_buffer = (int16_t*)realloc(wav, new_capacity * sizeof(int16_t));
            if (!new_buffer) {
                free(wav);
                free(buf);
                puts("Memory allocation error!");
                return -3;
            }
            wav = new_buffer;
            wav_capacity = new_capacity;
        }

        memcpy(wav + wav_size, work, frame_count * sizeof(int16_t));
        wav_size += frame_count;
    }
    printf("%zu samples generated.\n", wav_size);

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
    wh.dsize = wav_size * sizeof(int16_t);
    wh.fsize = wh.dsize + sizeof(wh) - 8;

    FILE* fw = fopen(argv[2], "wb");
    if (!fw) {
        free(wav);
        free(buf);
        puts("File open error!");
        return -1;
    }
    if (fwrite(&wh, 1, sizeof(wh), fw) != sizeof(wh)) {
        fclose(fw);
        free(wav);
        free(buf);
        puts("File write error!");
        return -1;
    }
    if (wav_size > 0) {
        size_t written = fwrite(wav, sizeof(int16_t), wav_size, fw);
        if (written != wav_size) {
            fclose(fw);
            free(wav);
            free(buf);
            puts("File write error!");
            return -1;
        }
    }
    fclose(fw);

    free(wav);
    free(buf);

    return 0;
}
