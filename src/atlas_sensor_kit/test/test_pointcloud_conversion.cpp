#include "atlas_sensor_kit/pointcloud_conversion.hpp"
#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_field.hpp>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace
{
using Cloud = sensor_msgs::msg::PointCloud2;
using Field = sensor_msgs::msg::PointField;
using atlas_sensor_kit::ConversionError;
using atlas_sensor_kit::livox_to_extended;
using atlas_sensor_kit::extended_to_compact;
template<typename T> void put(Cloud & c, size_t offset, T value)
{
  std::memcpy(c.data.data() + offset, &value, sizeof(T));
}
template<typename T> T get(const Cloud & c, size_t offset)
{
  T value;
  std::memcpy(&value, c.data.data() + offset, sizeof(T));
  return value;
}
Cloud input(size_t count = 2, int32_t seconds = 1700000000)
{
  Cloud c;
  c.header.frame_id = "test_sensor";
  c.header.stamp.sec = seconds;
  c.height = 1;
  c.width = count;
  c.point_step = 26;
  c.row_step = count * 26;
  c.is_dense = true;
  const std::array<const char *, 7> names{{"x", "y", "z", "intensity", "tag", "line", "timestamp"}};
  const std::array<uint32_t, 7> offsets{{0, 4, 8, 12, 16, 17, 18}};
  for (size_t i = 0; i < names.size(); ++i) {
    Field f;
    f.name = names[i]; f.offset = offsets[i]; f.count = 1;
    f.datatype = i < 4 ? Field::FLOAT32 : (i < 6 ? Field::UINT8 : Field::FLOAT64);
    c.fields.push_back(f);
  }
  c.data.resize(c.row_step);
  for (size_t i = 0; i < count; ++i) {
    put<float>(c, i * 26, static_cast<float>(i + 1));
    put<float>(c, i * 26 + 4, 2);
    put<float>(c, i * 26 + 8, 3);
    put<float>(c, i * 26 + 12, 12.5F);
    c.data[i * 26 + 16] = 255; c.data[i * 26 + 17] = 3;
    put<double>(c, i * 26 + 18, static_cast<double>(seconds) * 1e9 + i * 1024);
  }
  return c;
}

TEST(Conversion, ExtendedContractAndInputIndependence)
{
  const auto c = input();
  const auto before = c.data;
  const auto out = livox_to_extended(c);
  EXPECT_EQ(c.data, before);
  EXPECT_EQ(out.header, c.header);
  EXPECT_EQ(out.width, c.width); EXPECT_EQ(out.height, 1U);
  EXPECT_EQ(out.point_step, 32U); EXPECT_EQ(out.row_step, 64U);
  EXPECT_FALSE(out.is_bigendian);
  // Exact indexed contract checked by the pinned distortion corrector memory utility.
  const std::array<const char *, 10> names{{"x", "y", "z", "intensity", "return_type",
    "channel", "azimuth", "elevation", "distance", "time_stamp"}};
  const std::array<uint32_t, 10> offsets{{0, 4, 8, 12, 13, 14, 16, 20, 24, 28}};
  const std::array<uint8_t, 10> types{{Field::FLOAT32, Field::FLOAT32, Field::FLOAT32,
    Field::UINT8, Field::UINT8, Field::UINT16, Field::FLOAT32, Field::FLOAT32,
    Field::FLOAT32, Field::UINT32}};
  ASSERT_EQ(out.fields.size(), 10U);
  for (size_t i = 0; i < 10; ++i) {
    EXPECT_EQ(out.fields[i].name, names[i]); EXPECT_EQ(out.fields[i].offset, offsets[i]);
    EXPECT_EQ(out.fields[i].datatype, types[i]); EXPECT_EQ(out.fields[i].count, 1U);
  }
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_EQ(std::memcmp(c.data.data() + i * 26, out.data.data() + i * 32, 12), 0);
    EXPECT_EQ(out.data[i * 32 + 12], 13);
    EXPECT_EQ(out.data[i * 32 + 13], 0); EXPECT_EQ(get<uint16_t>(out, i * 32 + 14), 0);
    EXPECT_EQ(get<uint32_t>(out, i * 32 + 28), i * 1024);
  }
}

TEST(Conversion, CompactContractPreservesMetadataAndBits)
{
  auto c = livox_to_extended(input());
  put<uint8_t>(c, 13, 7); put<uint16_t>(c, 14, 513);
  put<uint8_t>(c, 44, 255); put<uint16_t>(c, 46, 65535);
  put<float>(c, 0, std::numeric_limits<float>::quiet_NaN());
  const auto before = c.data;
  const auto out = extended_to_compact(c);
  EXPECT_EQ(c.data, before); EXPECT_EQ(out.header, c.header);
  EXPECT_EQ(out.width, 2U); EXPECT_EQ(out.point_step, 16U);
  EXPECT_EQ(out.row_step, 32U); ASSERT_EQ(out.fields.size(), 6U);
  // CenterPoint InputPointType is exactly these six fields, with a 16-byte stride.
  for (size_t i = 0; i < 6; ++i) EXPECT_EQ(out.fields[i], c.fields[i]);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_EQ(std::memcmp(out.data.data() + i * 16, c.data.data() + i * 32, 16), 0);
  }
  EXPECT_EQ(get<uint16_t>(out, 14), 513); EXPECT_EQ(get<uint16_t>(out, 30), 65535);
}

TEST(Conversion, IntensityPolicy)
{
  const std::array<float, 9> values{{-1, 0, 12.49F, 12.5F, 254.5F, 300,
    std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
    -std::numeric_limits<float>::infinity()}};
  const std::array<uint8_t, 9> expected{{0, 0, 12, 13, 255, 255, 0, 0, 0}};
  auto c = input(values.size());
  for (size_t i = 0; i < values.size(); ++i) put<float>(c, i * 26 + 12, values[i]);
  const auto out = livox_to_extended(c);
  for (size_t i = 0; i < values.size(); ++i) EXPECT_EQ(out.data[i * 32 + 12], expected[i]);
}

TEST(Conversion, CartesianPolarConvention)
{
  const std::array<std::array<float, 3>, 5> xyz{{{{1, 0, 0}}, {{0, 1, 0}},
    {{0, -1, 0}}, {{0, 0, 1}}, {{0, 0, 0}}}};
  constexpr double pi = 3.14159265358979323846;
  const std::array<double, 5> az{{0, pi / 2, 3 * pi / 2, 0, 0}};
  auto c = input(xyz.size());
  for (size_t i = 0; i < xyz.size(); ++i) {
    for (size_t j = 0; j < 3; ++j) put<float>(c, i * 26 + j * 4, xyz[i][j]);
  }
  const auto out = livox_to_extended(c);
  for (size_t i = 0; i < xyz.size(); ++i) EXPECT_NEAR(get<float>(out, i * 32 + 16), az[i], 1e-6);
  EXPECT_NEAR(get<float>(out, 3 * 32 + 20), pi / 2, 1e-6);
  EXPECT_FLOAT_EQ(get<float>(out, 4 * 32 + 24), 0);
}

TEST(Conversion, TimestampValidationAndRounding)
{
  auto c = input(1, 1);
  c.header.stamp.nanosec = 123;
  put<double>(c, 18, 1000000123.5);
  EXPECT_EQ(get<uint32_t>(livox_to_extended(c), 28), 1U);
  for (double t : {1.0, 1000000.0, 1000000122.0,
      1000000123.0 + 4294967296.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()}) {
    put<double>(c, 18, t); EXPECT_THROW(livox_to_extended(c), ConversionError);
  }
  put<double>(c, 18, 1000000123.0 + 4294967295.0);
  EXPECT_THROW(livox_to_extended(c), ConversionError);
  put<double>(c, 18, 1000000123.0 + 4294967295.0 - 123.0);
  EXPECT_EQ(get<uint32_t>(livox_to_extended(c), 28), 4294967295U - 123U);
  c.header.stamp.nanosec = 0;
  put<double>(c, 18, 1000000000.0 + 4294967295.0);
  EXPECT_EQ(get<uint32_t>(livox_to_extended(c), 28), 4294967295U);
  auto decreasing = input();
  put<double>(decreasing, 18, 1700000000000001024.0);
  put<double>(decreasing, 44, 1700000000000000000.0);
  EXPECT_THROW(livox_to_extended(decreasing), ConversionError);
}

TEST(Conversion, AbsoluteDoubleNanosecondPrecision)
{
  constexpr uint64_t epoch = 1700000000000000000ULL;
  const double base = static_cast<double>(epoch);
  EXPECT_EQ(std::nextafter(base, std::numeric_limits<double>::infinity()) - base, 256.0);
  // A one-nanosecond difference has already disappeared before conversion.
  EXPECT_EQ(static_cast<double>(epoch + 1), base);
  auto c = input(2);
  put<double>(c, 18, static_cast<double>(epoch + 1));
  put<double>(c, 44, static_cast<double>(epoch + 129));
  const auto out = livox_to_extended(c);
  EXPECT_EQ(get<uint32_t>(out, 28), 0U); EXPECT_EQ(get<uint32_t>(out, 60), 256U);
  // Do not hide a represented timestamp earlier than the exact integer header.
  c.header.stamp.nanosec = 1;
  EXPECT_THROW(livox_to_extended(c), ConversionError);
}

TEST(Conversion, RejectMalformedClouds)
{
  const auto c = input();
  auto bad = c; bad.point_step = 32; EXPECT_THROW(livox_to_extended(bad), ConversionError);
  bad = c; bad.row_step++; EXPECT_THROW(livox_to_extended(bad), ConversionError);
  bad = c; bad.data.pop_back(); EXPECT_THROW(livox_to_extended(bad), ConversionError);
  bad = c; bad.data.push_back(0); EXPECT_THROW(livox_to_extended(bad), ConversionError);
  bad = c; bad.height = 2; EXPECT_THROW(livox_to_extended(bad), ConversionError);
  bad = c; bad.is_bigendian = true; EXPECT_THROW(livox_to_extended(bad), ConversionError);
  bad = c; bad.header.stamp.nanosec = 1000000000; EXPECT_THROW(livox_to_extended(bad), ConversionError);
  bad = c; bad.header.stamp.sec = -1; EXPECT_THROW(livox_to_extended(bad), ConversionError);
  for (size_t i = 0; i < c.fields.size(); ++i) {
    bad = c; bad.fields[i].offset++; EXPECT_THROW(livox_to_extended(bad), ConversionError);
    bad = c; bad.fields[i].datatype = 0; EXPECT_THROW(livox_to_extended(bad), ConversionError);
    bad = c; bad.fields[i].count = 2; EXPECT_THROW(livox_to_extended(bad), ConversionError);
    bad = c; bad.fields[i].name = "bad"; EXPECT_THROW(livox_to_extended(bad), ConversionError);
  }
  bad = c; bad.fields.pop_back(); EXPECT_THROW(livox_to_extended(bad), ConversionError);
  bad = c; put<float>(bad, 0, std::numeric_limits<float>::infinity());
  EXPECT_THROW(livox_to_extended(bad), ConversionError);
  bad = c; for (size_t j = 0; j < 3; ++j) put<float>(bad, j * 4, std::numeric_limits<float>::max());
  EXPECT_THROW(livox_to_extended(bad), ConversionError);

  const auto ext = livox_to_extended(c);
  for (size_t i = 0; i < ext.fields.size(); ++i) {
    bad = ext; bad.fields[i].offset++; EXPECT_THROW(extended_to_compact(bad), ConversionError);
    bad = ext; bad.fields[i].datatype = 0; EXPECT_THROW(extended_to_compact(bad), ConversionError);
  }
  bad = ext; bad.point_step = 16; EXPECT_THROW(extended_to_compact(bad), ConversionError);
  bad = ext; bad.data.pop_back(); EXPECT_THROW(extended_to_compact(bad), ConversionError);
  bad = ext; bad.row_step++; EXPECT_THROW(extended_to_compact(bad), ConversionError);
  bad = ext; bad.fields.pop_back(); EXPECT_THROW(extended_to_compact(bad), ConversionError);
  bad = ext; bad.fields[9].count = 2; EXPECT_THROW(extended_to_compact(bad), ConversionError);
  bad = ext; bad.fields[0].name = "bad"; EXPECT_THROW(extended_to_compact(bad), ConversionError);
  bad = ext; bad.is_bigendian = true; EXPECT_THROW(extended_to_compact(bad), ConversionError);
  bad = ext; bad.height = 2; EXPECT_THROW(extended_to_compact(bad), ConversionError);
  bad = c; bad.width = std::numeric_limits<uint32_t>::max();
  EXPECT_THROW(livox_to_extended(bad), ConversionError);
  bad = ext; bad.width = std::numeric_limits<uint32_t>::max();
  EXPECT_THROW(extended_to_compact(bad), ConversionError);
}

TEST(Conversion, EmptyClouds)
{
  const auto ext = livox_to_extended(input(0));
  const auto out = extended_to_compact(ext);
  EXPECT_EQ(out.width, 0U); EXPECT_TRUE(out.data.empty()); EXPECT_EQ(out.row_step, 0U);
}
}  // namespace
