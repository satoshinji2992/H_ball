#include "avi_mjpeg.hpp"

#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

int main(int argc, char **argv) {
    if (argc != 3) return 2;
    std::ifstream input(argv[1], std::ios::binary);
    const std::vector<char> jpeg((std::istreambuf_iterator<char>(input)),
                                 std::istreambuf_iterator<char>());
    if (jpeg.empty()) return 3;
    AviMjpegWriter writer;
    if (!writer.open(argv[2], 320, 240, 15)) return 4;
    for (int i = 0; i < 30; ++i) {
        if (!writer.write_frame(jpeg.data(), jpeg.size())) return 5;
    }
    writer.close();
    std::ifstream output(argv[2], std::ios::binary);
    char riff[4]{};
    output.read(riff, sizeof(riff));
    return std::string(riff, sizeof(riff)) == "RIFF" ? 0 : 6;
}
