#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include "vgmdrv.hpp"

namespace
{

struct WavHeader {
    std::array<char, 4> riff{'R', 'I', 'F', 'F'};
    std::uint32_t file_size{};
    std::array<char, 4> wave{'W', 'A', 'V', 'E'};
    std::array<char, 4> fmt{'f', 'm', 't', ' '};
    std::uint32_t fmt_size{16};
    std::uint16_t audio_format{1};
    std::uint16_t channels{};
    std::uint32_t sample_rate{};
    std::uint32_t byte_rate{};
    std::uint16_t block_align{};
    std::uint16_t bits_per_sample{16};
    std::array<char, 4> data{'d', 'a', 't', 'a'};
    std::uint32_t data_size{};
};

auto load_file(const std::filesystem::path& path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file) throw std::runtime_error("failed to open " + path.string());
    return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(file), {});
}

void write_wav(const std::filesystem::path& path, const std::vector<std::int16_t>& samples, int channels, int sample_rate)
{
    WavHeader header{};
    header.channels = static_cast<std::uint16_t>(channels);
    header.sample_rate = static_cast<std::uint32_t>(sample_rate);
    header.block_align = static_cast<std::uint16_t>(channels * sizeof(std::int16_t));
    header.byte_rate = header.sample_rate * header.block_align;
    header.data_size = static_cast<std::uint32_t>(samples.size() * sizeof(std::int16_t));
    header.file_size = header.data_size + sizeof(WavHeader) - 8;

    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("failed to open " + path.string());
    out.write(reinterpret_cast<const char*>(&header), sizeof(header));
    out.write(reinterpret_cast<const char*>(samples.data()), static_cast<std::streamsize>(samples.size() * sizeof(std::int16_t)));
    if (!out) throw std::runtime_error("failed to write wav data");
}

} // namespace

int main(int argc, char* argv[])
{
    try {
        if (argc < 3) {
            std::cerr << "usage: example /path/to/file.vgm /path/to/output.wav\n";
            return 1;
        }

        const std::filesystem::path input{argv[1]};
        const std::filesystem::path output{argv[2]};

        const auto data = load_file(input);

        constexpr int sampling_rate = 44100;
        constexpr int channels = 1;

        VgmDriver driver(sampling_rate, channels);
        if (!driver.load(data.data(), data.size())) {
            std::cerr << "Load failed\n";
            return 2;
        }

        std::vector<std::int16_t> audio;
        audio.reserve(1'000'000);
        std::array<std::int16_t, 4096> block{};

        while (driver.getLoopCount() == 0 && !driver.isEnded()) {
            driver.render(block.data(), static_cast<int>(block.size()));
            audio.insert(audio.end(), block.begin(), block.end());
        }

        std::cout << audio.size() << " samples generated.\n";
        std::cout << "Writing " << output << "...\n";
        write_wav(output, audio, channels, sampling_rate);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
}
