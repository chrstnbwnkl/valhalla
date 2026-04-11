#pragma once

// simple FIT file writer for garmin course files.
// header-only, no dependencies beyond the standard library.
// produces files compatible with garmin edge/fenix/forerunner devices.
//
// usage (in-memory):
//   fit::CourseWriter w;
//   w.set_course_name("morning loop");
//   w.set_sport(fit::Sport::cycling);
//   w.add_trackpoint({.lat = 50.9375, .lon = 6.9603, .alt = 55.0, .distance = 0.0, .timestamp =
//   ...}); w.add_trackpoint(...); w.add_course_point({.lat = 50.94, .lon = 6.96, .timestamp = ...,
//   .name = "turn", .type = fit::CoursePointType::left}); std::string fit_data = w.to_string();
//
// usage (file):
//   fit::CourseWriter w("my_ride.fit");
//   ... add trackpoints/course points ...
//   w.finish();

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fit {

// fit epoch is dec 31 1989 00:00:00 UTC
inline constexpr uint32_t garmin_epoch_offset = 631065600;

// convert a unix timestamp to fit timestamp
inline uint32_t to_fit_time(uint32_t unix_ts) {
  return unix_ts - garmin_epoch_offset;
}

// convert a unix timestamp from chrono
inline uint32_t to_fit_time(std::chrono::system_clock::time_point tp) {
  auto secs = std::chrono::duration_cast<std::chrono::seconds>(tp.time_since_epoch()).count();
  return static_cast<uint32_t>(secs) - garmin_epoch_offset;
}

// convert degrees to garmin semicircles
inline int32_t to_semicircles(double degrees) {
  return static_cast<int32_t>(degrees * (2147483648.0 / 180.0));
}

// altitude is stored as (meters + 500) * 5, as uint16
inline uint16_t to_fit_altitude(double meters) {
  return static_cast<uint16_t>((meters + 500.0) * 5.0);
}

// distance in meters * 100, stored as uint32
inline uint32_t to_fit_distance(double meters) {
  return static_cast<uint32_t>(meters * 100.0);
}

// speed in m/s * 1000, stored as uint16
inline uint16_t to_fit_speed(double mps) {
  return static_cast<uint16_t>(mps * 1000.0);
}

enum class Sport : uint8_t {
  generic = 0,
  running = 1,
  cycling = 2,
  hiking = 17,
  walking = 11,
  swimming = 5,
};

enum class CoursePointType : uint8_t {
  generic = 0,
  summit = 1,
  valley = 2,
  water = 3,
  food = 4,
  danger = 5,
  left = 6,
  right = 7,
  straight = 8,
  first_aid = 9,
  fourth_cat = 10,
  third_cat = 11,
  second_cat = 12,
  first_cat = 13,
  hors_cat = 14,
  sprint = 15,
  left_fork = 16,
  right_fork = 17,
  middle_fork = 18,
  slight_left = 19,
  sharp_left = 20,
  slight_right = 21,
  sharp_right = 22,
  u_turn = 23,
  segment_start = 24,
  segment_end = 25,
};

struct Trackpoint {
  double lat;         // degrees
  double lon;         // degrees
  double alt;         // meters
  double distance;    // cumulative meters from start
  uint32_t timestamp; // unix timestamp
};

struct CoursePoint {
  double lat;
  double lon;
  uint32_t timestamp; // unix timestamp
  double distance;    // cumulative meters from start
  std::string name;   // max ~16 chars for most devices
  CoursePointType type = CoursePointType::generic;
};

// fit base types
namespace base_type {
constexpr uint8_t uint8 = 0x00;
constexpr uint8_t sint8 = 0x01;
constexpr uint8_t uint16 = 0x84;
constexpr uint8_t sint16 = 0x83;
constexpr uint8_t uint32 = 0x86;
constexpr uint8_t sint32 = 0x85;
constexpr uint8_t string = 0x07;
constexpr uint8_t enum8 = 0x00;
} // namespace base_type

// global message numbers
namespace mesg {
constexpr uint16_t file_id = 0;
constexpr uint16_t event = 21;
constexpr uint16_t record = 20;
constexpr uint16_t lap = 19;
constexpr uint16_t course = 31;
constexpr uint16_t course_point = 32;
} // namespace mesg

class CourseWriter {
public:
  CourseWriter() = default;
  explicit CourseWriter(const std::string& filename) : filename_(filename) {
  }

  void set_course_name(const std::string& name) {
    course_name_ = name;
  }
  void set_sport(Sport sport) {
    sport_ = sport;
  }

  void add_trackpoint(const Trackpoint& tp) {
    trackpoints_.push_back(tp);
  }

  void add_course_point(const CoursePoint& cp) {
    course_points_.push_back(cp);
  }

  size_t num_trackpoints() const {
    return trackpoints_.size();
  }
  size_t num_course_points() const {
    return course_points_.size();
  }

  // build the FIT data and return it as a binary string
  std::string to_string() {
    if (trackpoints_.size() < 2)
      throw std::runtime_error("need at least 2 trackpoints for a course");

    std::ostringstream out(std::ios::binary);
    build(out);

    std::string data = out.str();

    // compute crc over all data
    uint16_t file_crc = 0;
    for (auto b : data)
      file_crc = crc16(file_crc, static_cast<uint8_t>(b));

    // append crc
    data.push_back(static_cast<char>(file_crc & 0xFF));
    data.push_back(static_cast<char>((file_crc >> 8) & 0xFF));
    return data;
  }

  // write the FIT data to a file
  void finish() {
    if (filename_.empty())
      throw std::runtime_error("no filename set");

    std::string data = to_string();
    std::ofstream out(filename_, std::ios::binary);
    if (!out)
      throw std::runtime_error("cannot open " + filename_);
    out.write(data.data(), data.size());
  }

private:
  std::string filename_;
  std::string course_name_ = "course";
  Sport sport_ = Sport::cycling;
  std::vector<Trackpoint> trackpoints_;
  std::vector<CoursePoint> course_points_;

  void build(std::ostream& out) {
    // write a placeholder header, we'll come back to fill in data_size
    auto header_pos = out.tellp();
    write_header(out, 0);

    // file_id message: type=course, manufacturer=development, product=0
    write_file_id(out);

    // course message: name and sport
    write_course(out);

    // timer start event
    write_event(out, trackpoints_.front().timestamp, 0 /* start */);

    // record messages (trackpoints)
    write_record_definition(out);
    for (auto& tp : trackpoints_)
      write_record_data(out, tp);

    // lap message (one lap covering the entire course)
    write_lap(out);

    // course points
    if (!course_points_.empty()) {
      write_course_point_definition(out);
      for (auto& cp : course_points_)
        write_course_point_data(out, cp);
    }

    // timer stop event
    write_event(out, trackpoints_.back().timestamp, 4 /* stop_all */);

    // compute data size and rewrite header
    auto end_pos = out.tellp();
    uint32_t data_size = static_cast<uint32_t>(end_pos) - 14; // 14 byte header
    out.seekp(header_pos);
    write_header(out, data_size);
    out.seekp(end_pos);
  }

  // crc lookup table per the fit spec
  static uint16_t crc16(uint16_t crc, uint8_t byte) {
    static const uint16_t table[16] = {0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00,
                                       0x2800, 0xE401, 0xA001, 0x6C00, 0x7800, 0xB401,
                                       0x5000, 0x9C01, 0x8801, 0x4400};
    uint16_t tmp;
    tmp = table[crc & 0xF];
    crc = (crc >> 4) & 0x0FFF;
    crc = crc ^ tmp ^ table[byte & 0xF];
    tmp = table[crc & 0xF];
    crc = (crc >> 4) & 0x0FFF;
    crc = crc ^ tmp ^ table[(byte >> 4) & 0xF];
    return crc;
  }

  // low level write helpers
  static void write_u8(std::ostream& out, uint8_t v) {
    out.write(reinterpret_cast<const char*>(&v), 1);
  }
  static void write_u16(std::ostream& out, uint16_t v) {
    out.write(reinterpret_cast<const char*>(&v), 2); // little endian
  }
  static void write_u32(std::ostream& out, uint32_t v) {
    out.write(reinterpret_cast<const char*>(&v), 4);
  }
  static void write_s32(std::ostream& out, int32_t v) {
    out.write(reinterpret_cast<const char*>(&v), 4);
  }
  static void write_str(std::ostream& out, const std::string& s, size_t len) {
    std::string padded = s;
    padded.resize(len, '\0');
    out.write(padded.data(), len);
  }

  // 14 byte file header
  void write_header(std::ostream& out, uint32_t data_size) {
    write_u8(out, 14);         // header size
    write_u8(out, 0x20);       // protocol version 2.0
    write_u16(out, 2132);      // profile version 21.32
    write_u32(out, data_size); // data size (excluding header + file crc)
    out.write(".FIT", 4);      // signature
    write_u16(out, 0x0000);    // header crc (0 = not used)
  }

  // definition message helper
  // record_header: 0x40 | local_msg_type
  struct FieldDef {
    uint8_t field_num;
    uint8_t size;
    uint8_t base_type;
  };

  void write_definition(std::ostream& out,
                        uint8_t local_msg,
                        uint16_t global_msg,
                        const std::vector<FieldDef>& fields) {
    write_u8(out, 0x40 | (local_msg & 0x0F)); // definition header
    write_u8(out, 0);                         // reserved
    write_u8(out, 0);                         // architecture: little endian
    write_u16(out, global_msg);
    write_u8(out, static_cast<uint8_t>(fields.size()));
    for (auto& f : fields) {
      write_u8(out, f.field_num);
      write_u8(out, f.size);
      write_u8(out, f.base_type);
    }
  }

  // file_id: type=6(course), manufacturer=255(development), product=0, serial=12345, time_created
  void write_file_id(std::ostream& out) {
    write_definition(out, 0, mesg::file_id,
                     {
                         {0, 1, base_type::enum8},  // type
                         {1, 2, base_type::uint16}, // manufacturer
                         {2, 2, base_type::uint16}, // product
                         {3, 4, base_type::uint32}, // serial_number
                         {4, 4, base_type::uint32}, // time_created
                     });
    write_u8(out, 0);                                            // data header: local msg 0
    write_u8(out, 6);                                            // type = course
    write_u16(out, 255);                                         // manufacturer = development
    write_u16(out, 0);                                           // product
    write_u32(out, 12345);                                       // serial number
    write_u32(out, to_fit_time(trackpoints_.front().timestamp)); // time created
  }

  // course message: name + sport
  void write_course(std::ostream& out) {
    uint8_t name_len =
        std::max<uint8_t>(16, static_cast<uint8_t>(std::min<size_t>(course_name_.size() + 1, 32)));
    write_definition(out, 1, mesg::course,
                     {
                         {4, 1, base_type::enum8},         // sport
                         {5, name_len, base_type::string}, // course name
                     });
    write_u8(out, 1); // data header: local msg 1
    write_u8(out, static_cast<uint8_t>(sport_));
    write_str(out, course_name_, name_len);
  }

  // event message: timer start/stop
  void write_event(std::ostream& out, uint32_t unix_ts, uint8_t event_type) {
    write_definition(out, 2, mesg::event,
                     {
                         {253, 4, base_type::uint32}, // timestamp
                         {0, 1, base_type::enum8},    // event (0 = timer)
                         {1, 1, base_type::enum8},    // event_type
                     });
    write_u8(out, 2); // data header: local msg 2
    write_u32(out, to_fit_time(unix_ts));
    write_u8(out, 0); // event = timer
    write_u8(out, event_type);
  }

  // record definition (trackpoints)
  void write_record_definition(std::ostream& out) {
    write_definition(out, 3, mesg::record,
                     {
                         {253, 4, base_type::uint32}, // timestamp
                         {0, 4, base_type::sint32},   // position_lat
                         {1, 4, base_type::sint32},   // position_long
                         {2, 2, base_type::uint16},   // altitude
                         {5, 4, base_type::uint32},   // distance
                     });
  }

  void write_record_data(std::ostream& out, const Trackpoint& tp) {
    write_u8(out, 3); // data header: local msg 3
    write_u32(out, to_fit_time(tp.timestamp));
    write_s32(out, to_semicircles(tp.lat));
    write_s32(out, to_semicircles(tp.lon));
    write_u16(out, to_fit_altitude(tp.alt));
    write_u32(out, to_fit_distance(tp.distance));
  }

  // lap message covering the whole course
  void write_lap(std::ostream& out) {
    auto& first = trackpoints_.front();
    auto& last = trackpoints_.back();

    uint32_t elapsed = to_fit_time(last.timestamp) - to_fit_time(first.timestamp);
    // elapsed_time and timer_time in milliseconds
    uint32_t elapsed_ms = elapsed * 1000;

    write_definition(out, 4, mesg::lap,
                     {
                         {254, 2, base_type::uint16}, // message_index
                         {253, 4, base_type::uint32}, // timestamp
                         {2, 4, base_type::uint32},   // start_time
                         {3, 4, base_type::sint32},   // start_position_lat
                         {4, 4, base_type::sint32},   // start_position_long
                         {5, 4, base_type::sint32},   // end_position_lat
                         {6, 4, base_type::sint32},   // end_position_long
                         {7, 4, base_type::uint32},   // total_elapsed_time (ms)
                         {8, 4, base_type::uint32},   // total_timer_time (ms)
                         {9, 4, base_type::uint32},   // total_distance (cm)
                     });
    write_u8(out, 4);
    write_u16(out, 0); // message_index = 0 (single lap)
    write_u32(out, to_fit_time(last.timestamp));
    write_u32(out, to_fit_time(first.timestamp));
    write_s32(out, to_semicircles(first.lat));
    write_s32(out, to_semicircles(first.lon));
    write_s32(out, to_semicircles(last.lat));
    write_s32(out, to_semicircles(last.lon));
    write_u32(out, elapsed_ms);
    write_u32(out, elapsed_ms);
    write_u32(out, to_fit_distance(last.distance));
  }

  void write_course_point_definition(std::ostream& out) {
    write_definition(out, 5, mesg::course_point,
                     {
                         {1, 4, base_type::uint32},  // timestamp
                         {2, 4, base_type::sint32},  // position_lat
                         {3, 4, base_type::sint32},  // position_long
                         {4, 4, base_type::uint32},  // distance
                         {5, 1, base_type::enum8},   // type
                         {6, 16, base_type::string}, // name
                     });
  }

  void write_course_point_data(std::ostream& out, const CoursePoint& cp) {
    write_u8(out, 5);
    write_u32(out, to_fit_time(cp.timestamp));
    write_s32(out, to_semicircles(cp.lat));
    write_s32(out, to_semicircles(cp.lon));
    write_u32(out, to_fit_distance(cp.distance));
    write_u8(out, static_cast<uint8_t>(cp.type));
    write_str(out, cp.name, 16);
  }
};

} // namespace fit