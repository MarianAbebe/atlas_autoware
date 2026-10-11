#include "atlas_sensor_kit/pointcloud_conversion.hpp"

#include <sensor_msgs/msg/point_field.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#ifdef ATLAS_HAS_AUTOWARE_POINT_TYPES
#include <autoware/point_types/memory.hpp>
#endif

namespace atlas_sensor_kit
{
namespace
{
static_assert(std::numeric_limits<long double>::digits >= 64,
  "timestamp subtraction requires exact integer header nanoseconds");
using Cloud = sensor_msgs::msg::PointCloud2;
using Field = sensor_msgs::msg::PointField;
struct Spec {const char * name; uint32_t offset; uint8_t type;};
constexpr std::array<Spec, 7> livox{{
  {"x", 0, Field::FLOAT32}, {"y", 4, Field::FLOAT32}, {"z", 8, Field::FLOAT32},
  {"intensity", 12, Field::FLOAT32}, {"tag", 16, Field::UINT8},
  {"line", 17, Field::UINT8}, {"timestamp", 18, Field::FLOAT64}}};
constexpr std::array<Spec, 10> extended{{
  {"x", 0, Field::FLOAT32}, {"y", 4, Field::FLOAT32}, {"z", 8, Field::FLOAT32},
  {"intensity", 12, Field::UINT8}, {"return_type", 13, Field::UINT8},
  {"channel", 14, Field::UINT16}, {"azimuth", 16, Field::FLOAT32},
  {"elevation", 20, Field::FLOAT32}, {"distance", 24, Field::FLOAT32},
  {"time_stamp", 28, Field::UINT32}}};

template<size_t N>
void validate(const Cloud & cloud, const std::array<Spec, N> & specs, uint32_t stride)
{
  if (cloud.is_bigendian) throw ConversionError("big-endian input is unsupported");
  if (cloud.header.stamp.sec < 0 || cloud.header.stamp.nanosec >= 1000000000U) {
    throw ConversionError("header stamp must be nonnegative with normalized nanoseconds");
  }
  if (cloud.height != 1) throw ConversionError("expected unorganized cloud (height=1)");
  if (cloud.point_step != stride) throw ConversionError("incorrect point_step");
  const uint64_t bytes = uint64_t{cloud.width} * stride;
  if (bytes != cloud.row_step || bytes != cloud.data.size()) {
    throw ConversionError("row_step/data size do not match width and point_step");
  }
  if (cloud.fields.size() != N) throw ConversionError("incorrect number of point fields");
  for (size_t i = 0; i < N; ++i) {
    const auto & f = cloud.fields[i];
    const auto & s = specs[i];
    if (f.name != s.name || f.offset != s.offset || f.datatype != s.type || f.count != 1) {
      throw ConversionError("invalid field " + std::string{s.name} + " at index " + std::to_string(i));
    }
  }
  const uint16_t endian_probe = 1;
  if (*reinterpret_cast<const uint8_t *>(&endian_probe) != 1) {
    throw ConversionError("adapter requires a little-endian host");
  }
}

template<typename T> T read(const uint8_t * p)
{
  T value;
  std::memcpy(&value, p, sizeof(T));
  return value;
}
template<typename T> void write(uint8_t * p, T value)
{
  std::memcpy(p, &value, sizeof(T));
}
Cloud output_cloud(const Cloud & input, uint32_t stride, size_t field_count)
{
  Cloud out;
  out.header = input.header;
  out.height = 1;
  out.width = input.width;
  out.point_step = stride;
  const uint64_t bytes = uint64_t{out.width} * stride;
  if (bytes > std::numeric_limits<uint32_t>::max()) throw ConversionError("output row_step overflow");
  out.row_step = static_cast<uint32_t>(bytes);
  out.is_dense = input.is_dense;
  for (size_t i = 0; i < field_count; ++i) {
    Field f;
    f.name = extended[i].name;
    f.offset = extended[i].offset;
    f.datatype = extended[i].type;
    f.count = 1;
    out.fields.push_back(f);
  }
#ifdef ATLAS_HAS_AUTOWARE_POINT_TYPES
  out.fields = field_count == 10 ? autoware::point_types::create_fields_point_xyzircaedt() :
    autoware::point_types::create_fields_point_xyzirc();
#endif
  out.data.resize(bytes);
  return out;
}
}  // namespace

Cloud livox_to_extended(const Cloud & input)
{
  validate(input, livox, 26);
  auto out = output_cloud(input, 32, 10);
  // Integer header time is retained exactly in long double on the target Linux platforms.
  const long double base = static_cast<long double>(input.header.stamp.sec) * 1000000000.0L +
    input.header.stamp.nanosec;
  long double previous = -1;
  constexpr double pi = 3.14159265358979323846;
  for (size_t i = 0; i < input.width; ++i) {
    const auto * src = input.data.data() + i * 26;
    auto * dst = out.data.data() + i * 32;
    const double x = read<float>(src), y = read<float>(src + 4), z = read<float>(src + 8);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      throw ConversionError("nonfinite XYZ at point " + std::to_string(i));
    }
    const double distance = std::hypot(std::hypot(x, y), z);
    if (distance > std::numeric_limits<float>::max()) {
      throw ConversionError("distance overflows FLOAT32 at point " + std::to_string(i));
    }
    const double timestamp = read<double>(src + 18);
    const long double offset = static_cast<long double>(timestamp) - base;
    if (!std::isfinite(timestamp) || offset < 0 ||
      offset > std::numeric_limits<uint32_t>::max()) {
      throw ConversionError("timestamp must be absolute nanoseconds within UINT32 of header at point " +
        std::to_string(i));
    }
    if (offset < previous) throw ConversionError("timestamps decrease at point " + std::to_string(i));
    previous = offset;
    const long double rounded = std::round(offset);
    if (rounded > std::numeric_limits<uint32_t>::max()) throw ConversionError("rounded timestamp overflow");
    // Universe 0.52.0 adds these two UINT32 values before converting to seconds.
    // Reject offsets that would silently wrap in the pinned distortion corrector.
    if (rounded > std::numeric_limits<uint32_t>::max() - input.header.stamp.nanosec) {
      throw ConversionError("header nanoseconds plus point offset overflow pinned deskewer at point " +
        std::to_string(i));
    }
    std::memcpy(dst, src, 12);  // preserve XYZ bit patterns
    const float intensity = read<float>(src + 12);
    dst[12] = std::isfinite(intensity) ?
      static_cast<uint8_t>(std::round(std::clamp(intensity, 0.0F, 255.0F))) : 0;
    // Provisional INVALID return type and channel zero; tag/line are deliberately not mapped.
    dst[13] = 0;
    write<uint16_t>(dst + 14, 0);
    double azimuth = (x == 0 && y == 0) ? 0 : std::atan2(y, x);
    if (azimuth < 0) azimuth += 2 * pi;
    float azimuth_f = static_cast<float>(azimuth);
    if (azimuth_f >= 2 * pi) azimuth_f = 0;
    write<float>(dst + 16, azimuth_f);
    write<float>(dst + 20, distance == 0 ? 0.0F :
      static_cast<float>(std::atan2(z, std::hypot(x, y))));
    write<float>(dst + 24, static_cast<float>(distance));
    write<uint32_t>(dst + 28, static_cast<uint32_t>(rounded));
  }
#ifdef ATLAS_HAS_AUTOWARE_POINT_TYPES
  if (!autoware::point_types::is_data_layout_compatible_with_point_xyzircaedt(out)) {
    throw ConversionError("internal XYZIRCAEDT contract error");
  }
#endif
  return out;
}

Cloud extended_to_compact(const Cloud & input)
{
  validate(input, extended, 32);
  auto out = output_cloud(input, 16, 6);
  for (size_t i = 0; i < input.width; ++i) {
    std::memcpy(out.data.data() + i * 16, input.data.data() + i * 32, 16);
  }
#ifdef ATLAS_HAS_AUTOWARE_POINT_TYPES
  if (!autoware::point_types::is_data_layout_compatible_with_point_xyzirc(out)) {
    throw ConversionError("internal XYZIRC contract error");
  }
#endif
  return out;
}
}  // namespace atlas_sensor_kit
