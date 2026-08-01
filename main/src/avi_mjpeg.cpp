#include "avi_mjpeg.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

AviMjpegWriter::~AviMjpegWriter() { close(); }

void AviMjpegWriter::write_fourcc(const char value[4]) { file_.write(value, 4); }

void AviMjpegWriter::write_u16(uint16_t value) {
    const char bytes[2] = {static_cast<char>(value & 0xff),
                           static_cast<char>((value >> 8) & 0xff)};
    file_.write(bytes, sizeof(bytes));
}

void AviMjpegWriter::write_u32(uint32_t value) {
    const char bytes[4] = {static_cast<char>(value & 0xff),
                           static_cast<char>((value >> 8) & 0xff),
                           static_cast<char>((value >> 16) & 0xff),
                           static_cast<char>((value >> 24) & 0xff)};
    file_.write(bytes, sizeof(bytes));
}

void AviMjpegWriter::patch_u32(std::streampos position, uint32_t value) {
    const std::streampos current = file_.tellp();
    file_.seekp(position);
    write_u32(value);
    file_.seekp(current);
}

bool AviMjpegWriter::open(const std::string &path, int width, int height, int fps) {
    close();
    file_.open(path, std::ios::binary | std::ios::trunc);
    if (!file_) return false;
    path_ = path;
    width_ = width;
    height_ = height;
    fps_ = std::max(1, fps);
    frame_count_ = 0;
    max_frame_size_ = 0;
    index_.clear();

    write_fourcc("RIFF"); riff_size_pos_ = file_.tellp(); write_u32(0); write_fourcc("AVI ");
    write_fourcc("LIST"); const std::streampos hdrl_size_pos = file_.tellp(); write_u32(0);
    const std::streampos hdrl_start = file_.tellp(); write_fourcc("hdrl");

    write_fourcc("avih"); write_u32(56);
    write_u32(static_cast<uint32_t>(1000000 / fps_));
    write_u32(0); write_u32(0); write_u32(0x10);
    total_frames_pos_ = file_.tellp(); write_u32(0);
    write_u32(0); write_u32(1);
    suggested_buffer_pos_ = file_.tellp(); write_u32(0);
    write_u32(static_cast<uint32_t>(width_)); write_u32(static_cast<uint32_t>(height_));
    write_u32(0); write_u32(0); write_u32(0); write_u32(0);

    write_fourcc("LIST"); const std::streampos strl_size_pos = file_.tellp(); write_u32(0);
    const std::streampos strl_start = file_.tellp(); write_fourcc("strl");
    write_fourcc("strh"); write_u32(56); write_fourcc("vids"); write_fourcc("MJPG");
    write_u32(0); write_u16(0); write_u16(0); write_u32(0); write_u32(1);
    write_u32(static_cast<uint32_t>(fps_)); write_u32(0);
    stream_length_pos_ = file_.tellp(); write_u32(0);
    stream_buffer_pos_ = file_.tellp(); write_u32(0);
    write_u32(0xffffffffU); write_u32(0);
    write_u16(0); write_u16(0); write_u16(static_cast<uint16_t>(width_));
    write_u16(static_cast<uint16_t>(height_));

    write_fourcc("strf"); write_u32(40); write_u32(40);
    write_u32(static_cast<uint32_t>(width_)); write_u32(static_cast<uint32_t>(height_));
    write_u16(1); write_u16(24); write_fourcc("MJPG");
    write_u32(static_cast<uint32_t>(width_ * height_ * 3));
    write_u32(0); write_u32(0); write_u32(0); write_u32(0);

    const std::streampos after_strl = file_.tellp();
    patch_u32(strl_size_pos, static_cast<uint32_t>(after_strl - strl_start));
    patch_u32(hdrl_size_pos, static_cast<uint32_t>(after_strl - hdrl_start));

    write_fourcc("LIST"); movi_size_pos_ = file_.tellp(); write_u32(0);
    movi_fourcc_pos_ = file_.tellp(); write_fourcc("movi");
    return static_cast<bool>(file_);
}

bool AviMjpegWriter::write_frame(const void *jpeg, size_t size) {
    if (!file_ || !jpeg || size == 0 || size > 0xffffffffU) return false;
    const std::streampos chunk_pos = file_.tellp();
    write_fourcc("00dc"); write_u32(static_cast<uint32_t>(size));
    file_.write(static_cast<const char *>(jpeg), static_cast<std::streamsize>(size));
    if (size & 1U) file_.put('\0');
    if (!file_) return false;
    index_.push_back({static_cast<uint32_t>(chunk_pos - movi_fourcc_pos_),
                      static_cast<uint32_t>(size)});
    ++frame_count_;
    max_frame_size_ = std::max(max_frame_size_, static_cast<uint32_t>(size));
    return true;
}

void AviMjpegWriter::close() {
    if (!file_.is_open()) return;
    const std::streampos idx_pos = file_.tellp();
    patch_u32(movi_size_pos_, static_cast<uint32_t>(idx_pos - movi_fourcc_pos_));
    write_fourcc("idx1"); write_u32(static_cast<uint32_t>(index_.size() * 16U));
    for (const IndexEntry &entry : index_) {
        write_fourcc("00dc"); write_u32(0x10); write_u32(entry.offset); write_u32(entry.size);
    }
    patch_u32(total_frames_pos_, frame_count_);
    patch_u32(stream_length_pos_, frame_count_);
    patch_u32(suggested_buffer_pos_, max_frame_size_);
    patch_u32(stream_buffer_pos_, max_frame_size_);
    const std::streampos end = file_.tellp();
    patch_u32(riff_size_pos_, static_cast<uint32_t>(end - std::streamoff(8)));
    file_.flush();
    file_.close();
}

std::string make_recording_filename() {
    const auto now_point = std::chrono::system_clock::now();
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
        now_point.time_since_epoch()).count() % 1000;
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    std::ostringstream out;
    out << "h_ball_" << std::put_time(&local, "%Y%m%d_%H%M%S") << '_'
        << std::setw(3) << std::setfill('0') << millis << ".avi";
    return out.str();
}
