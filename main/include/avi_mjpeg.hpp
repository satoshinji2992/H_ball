#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

class AviMjpegWriter {
public:
    AviMjpegWriter() = default;
    ~AviMjpegWriter();

    bool open(const std::string &path, int width, int height, int fps);
    bool write_frame(const void *jpeg, size_t size);
    void close();
    bool is_open() const { return file_.is_open(); }
    const std::string &path() const { return path_; }
    uint32_t frame_count() const { return frame_count_; }

private:
    struct IndexEntry {
        uint32_t offset;
        uint32_t size;
    };

    void write_fourcc(const char value[4]);
    void write_u16(uint16_t value);
    void write_u32(uint32_t value);
    void patch_u32(std::streampos position, uint32_t value);

    std::ofstream file_;
    std::string path_;
    int width_ = 0;
    int height_ = 0;
    int fps_ = 15;
    uint32_t frame_count_ = 0;
    uint32_t max_frame_size_ = 0;
    std::streampos riff_size_pos_{};
    std::streampos total_frames_pos_{};
    std::streampos stream_length_pos_{};
    std::streampos suggested_buffer_pos_{};
    std::streampos stream_buffer_pos_{};
    std::streampos movi_size_pos_{};
    std::streampos movi_fourcc_pos_{};
    std::vector<IndexEntry> index_;
};

std::string make_recording_filename();
